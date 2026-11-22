/**
 * @file trt_engine.cpp
 * @brief Implementation of the TensorRT engine wrapper.
 * @details Handles engine deserialization, binding inspection, FP32/FP16
 * staging buffers, CUDA copies and `enqueueV2` execution.
 *
 * @copyright MIT License, 2026.
 */

#include "astra_rgbd_perception/trt_engine.hpp"

#include <ros/ros.h>

#include <algorithm>
#include <fstream>
#include <sstream>

namespace astra_rgbd_perception {

/**
 * @brief Forward TensorRT log messages to ROS.
 * @param severity TensorRT severity level.
 * @param msg Null-terminated message.
 */
void TrtLogger::log(Severity severity, const char* msg) noexcept {
  if (severity == Severity::kINTERNAL_ERROR || severity == Severity::kERROR) {
    ROS_ERROR("[TensorRT] %s", msg);
  } else if (severity == Severity::kWARNING) {
    ROS_WARN("[TensorRT] %s", msg);
  } else {
    ROS_DEBUG("[TensorRT] %s", msg);
  }
}

/// @brief Construct an empty engine wrapper.
TrtEngine::TrtEngine() = default;

/// @brief Release all resources owned by the engine wrapper.
TrtEngine::~TrtEngine() { release(); }

/**
 * @brief Compute the total number of elements in a TensorRT dimensions object.
 * @param dims TensorRT dimensions.
 * @return Product of all dimensions, or 0 if any dimension is unresolved.
 */
size_t TrtEngine::volume(const nvinfer1::Dims& dims) {
  size_t result = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      return 0;
    }
    result *= static_cast<size_t>(dims.d[i]);
  }
  return result;
}

/**
 * @brief Deserialize a TensorRT engine and allocate its CUDA resources.
 * @param engine_path Absolute path to the serialized engine.
 * @return `true` on success.
 *
 * Steps:
 * 1. Read the engine blob.
 * 2. Create runtime/engine/context.
 * 3. Resolve input/output binding dimensions and dtypes.
 * 4. Allocate device buffers and host staging buffers.
 */
bool TrtEngine::load(const std::string& engine_path) {
  release();

  std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
  if (!file) {
    ROS_ERROR("Cannot open TensorRT engine: %s", engine_path.c_str());
    return false;
  }
  const std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<char> blob(static_cast<size_t>(size));
  if (!file.read(blob.data(), size)) {
    ROS_ERROR("Cannot read TensorRT engine: %s", engine_path.c_str());
    return false;
  }

  runtime_ = nvinfer1::createInferRuntime(logger_);
  if (runtime_ == nullptr) {
    ROS_ERROR("createInferRuntime failed");
    return false;
  }
  engine_ = runtime_->deserializeCudaEngine(blob.data(), blob.size(), nullptr);
  if (engine_ == nullptr) {
    ROS_ERROR("deserializeCudaEngine failed (engine/device mismatch?)");
    return false;
  }
  context_ = engine_->createExecutionContext();
  if (context_ == nullptr) {
    ROS_ERROR("createExecutionContext failed");
    return false;
  }

  for (int i = 0; i < engine_->getNbBindings(); ++i) {
    if (engine_->bindingIsInput(i)) {
      input_index_ = i;
    } else {
      output_index_ = i;
    }
    const nvinfer1::Dims dims = engine_->getBindingDimensions(i);
    std::ostringstream dims_text;
    for (int d = 0; d < dims.nbDims; ++d) {
      dims_text << (d ? "x" : "") << dims.d[d];
    }
    ROS_INFO("binding %d: name=%s input=%d dtype=%d dims=[%s]", i,
             engine_->getBindingName(i),
             static_cast<int>(engine_->bindingIsInput(i)),
             static_cast<int>(engine_->getBindingDataType(i)),
             dims_text.str().c_str());
  }
  if (engine_->getNbBindings() != 2 || input_index_ < 0 || output_index_ < 0) {
    ROS_ERROR("Engine does not have exactly one input and one output");
    return false;
  }

  nvinfer1::Dims input_dims = engine_->getBindingDimensions(input_index_);
  if (input_dims.d[0] < 0) {
    input_dims.d[0] = 1;
    if (!context_->setBindingDimensions(input_index_, input_dims)) {
      ROS_ERROR("setBindingDimensions failed for input");
      return false;
    }
  }
  nvinfer1::Dims output_dims = context_->getBindingDimensions(output_index_);
  if (output_dims.d[0] < 0) {
    output_dims.d[0] = 1;
  }

  input_dims_.assign(input_dims.d, input_dims.d + input_dims.nbDims);
  output_dims_.assign(output_dims.d, output_dims.d + output_dims.nbDims);
  input_elements_ = volume(input_dims);
  output_elements_ = volume(output_dims);
  if (input_elements_ == 0 || output_elements_ == 0) {
    ROS_ERROR("Engine has unresolved dynamic dimensions");
    return false;
  }

  const auto input_type = engine_->getBindingDataType(input_index_);
  const auto output_type = engine_->getBindingDataType(output_index_);
  using DataType = nvinfer1::DataType;
  if ((input_type != DataType::kFLOAT && input_type != DataType::kHALF) ||
      (output_type != DataType::kFLOAT && output_type != DataType::kHALF) ||
      engine_->getBindingFormat(input_index_) != nvinfer1::TensorFormat::kLINEAR ||
      engine_->getBindingFormat(output_index_) != nvinfer1::TensorFormat::kLINEAR) {
    ROS_ERROR("Engine bindings must use linear FP32 or FP16 tensors");
    return false;
  }

  const bool input_half = input_type == DataType::kHALF;
  const bool output_half = output_type == DataType::kHALF;
  input_bytes_ = input_elements_ * (input_half ? sizeof(__half) : sizeof(float));
  output_bytes_ = output_elements_ * (output_half ? sizeof(__half) : sizeof(float));
  if (input_half) {
    host_input_half_.resize(input_elements_);
  }
  if (output_half) {
    host_output_half_.resize(output_elements_);
  } else {
    host_output_.resize(output_elements_);
  }

  if (cudaMalloc(&device_input_, input_bytes_) != cudaSuccess ||
      cudaMalloc(&device_output_, output_bytes_) != cudaSuccess) {
    ROS_ERROR("cudaMalloc failed for engine buffers");
    return false;
  }
  if (cudaStreamCreate(&stream_) != cudaSuccess) {
    ROS_ERROR("cudaStreamCreate failed");
    return false;
  }

  std::ostringstream info;
  info << "Loaded " << engine_path << " | input "
       << engine_->getBindingName(input_index_) << " [";
  for (size_t i = 0; i < input_dims_.size(); ++i) {
    info << (i ? "x" : "") << input_dims_[i];
  }
  info << "] | output " << engine_->getBindingName(output_index_) << " [";
  for (size_t i = 0; i < output_dims_.size(); ++i) {
    info << (i ? "x" : "") << output_dims_[i];
  }
  info << "]";
  ROS_INFO_STREAM(info.str());
  return true;
}

/**
 * @brief Run one synchronous inference with host FP32 input/output.
 * @param input Host FP32 NCHW input tensor.
 * @param input_elements Element count; must match the engine input binding.
 * @param output Output vector resized to the engine output size.
 * @return `true` on success.
 *
 * If the input binding is FP16, the input is converted to `__half` first.
 * If the output binding is FP16, the output is converted back to FP32.
 */
bool TrtEngine::infer(const float* input, size_t input_elements,
                      std::vector<float>* output) {
  if (!ok() || input == nullptr || output == nullptr) {
    return false;
  }
  if (input_elements != input_elements_) {
    ROS_ERROR("Input element count mismatch: got %zu expected %zu",
              input_elements, input_elements_);
    return false;
  }

  const void* input_data = input;
  if (!host_input_half_.empty()) {
    std::transform(input, input + input_elements_, host_input_half_.begin(),
                   [](float value) { return __float2half_rn(value); });
    input_data = host_input_half_.data();
  }
  if (cudaMemcpyAsync(device_input_, input_data, input_bytes_,
                      cudaMemcpyHostToDevice, stream_) != cudaSuccess) {
    ROS_ERROR("cudaMemcpyAsync H2D failed");
    return false;
  }

  std::vector<void*> bindings(static_cast<size_t>(engine_->getNbBindings()),
                              nullptr);
  bindings[input_index_] = device_input_;
  bindings[output_index_] = device_output_;
  if (!context_->enqueueV2(bindings.data(), stream_, nullptr)) {
    ROS_ERROR("TensorRT enqueueV2 failed");
    return false;
  }
  const cudaError_t enqueue_error = cudaGetLastError();
  if (enqueue_error != cudaSuccess) {
    ROS_ERROR("CUDA error after enqueueV2: %s",
              cudaGetErrorString(enqueue_error));
    return false;
  }
  void* output_data = host_output_half_.empty()
                          ? static_cast<void*>(host_output_.data())
                          : static_cast<void*>(host_output_half_.data());
  if (cudaMemcpyAsync(output_data, device_output_, output_bytes_,
                      cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
    ROS_ERROR("cudaMemcpyAsync D2H failed");
    return false;
  }
  const cudaError_t sync_error = cudaStreamSynchronize(stream_);
  if (sync_error != cudaSuccess) {
    ROS_ERROR("cudaStreamSynchronize failed: %s",
              cudaGetErrorString(sync_error));
    return false;
  }

  if (host_output_half_.empty()) {
    output->assign(host_output_.begin(), host_output_.end());
  } else {
    output->resize(output_elements_);
    std::transform(host_output_half_.begin(), host_output_half_.end(),
                   output->begin(), [](__half value) { return __half2float(value); });
  }
  return true;
}

/// @brief Destroy CUDA buffers, the stream and all TensorRT objects.
void TrtEngine::release() {
  if (stream_ != nullptr) {
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
  if (device_input_ != nullptr) {
    cudaFree(device_input_);
    device_input_ = nullptr;
  }
  if (device_output_ != nullptr) {
    cudaFree(device_output_);
    device_output_ = nullptr;
  }
  if (context_ != nullptr) {
    context_->destroy();
    context_ = nullptr;
  }
  if (engine_ != nullptr) {
    engine_->destroy();
    engine_ = nullptr;
  }
  if (runtime_ != nullptr) {
    runtime_->destroy();
    runtime_ = nullptr;
  }
  input_index_ = -1;
  output_index_ = -1;
  input_dims_.clear();
  output_dims_.clear();
  input_elements_ = 0;
  output_elements_ = 0;
  input_bytes_ = 0;
  output_bytes_ = 0;
  host_input_half_.clear();
  host_output_half_.clear();
  host_output_.clear();
}

}  // namespace astra_rgbd_perception
