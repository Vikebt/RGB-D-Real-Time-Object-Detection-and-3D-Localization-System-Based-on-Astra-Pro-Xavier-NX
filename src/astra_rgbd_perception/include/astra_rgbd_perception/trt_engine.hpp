/**
 * @file trt_engine.hpp
 * @brief Minimal TensorRT 8.x engine wrapper for the YOLOv5n detector.
 * @details
 * The wrapper owns a serialized TensorRT engine, one execution context and a
 * CUDA stream. It supports a single linear input and a single linear output
 * binding in either FP32 or FP16. Host-side float data is converted to
 * `__half` when the binding is FP16.
 *
 * The class is intentionally small: model-specific preprocessing and YOLO
 * post-processing live in `detector_node.cpp`, not here.
 *
 * @note The engine is device- and TensorRT-version-specific. Rebuild it on the
 *       target Jetson after changing hardware or TensorRT.
 *
 * @copyright MIT License, 2026.
 */

#pragma once

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <NvInfer.h>

#include <string>
#include <vector>

/**
 * @namespace astra_rgbd_perception
 * @brief RGB-D perception nodes for Astra Pro on Jetson Xavier NX.
 */
namespace astra_rgbd_perception {

/**
 * @class TrtLogger
 * @brief TensorRT logger that forwards messages to ROS logging.
 */
class TrtLogger : public nvinfer1::ILogger {
 public:
  /**
   * @brief Receive one TensorRT log message.
   * @param severity TensorRT severity level.
   * @param msg Null-terminated message text.
   */
  void log(Severity severity, const char* msg) noexcept override;
};

/**
 * @class TrtEngine
 * @brief Single-input/single-output TensorRT engine wrapper.
 *
 * Typical use:
 * @code
 * TrtEngine engine;
 * if (!engine.load("/path/yolov5n_fp16.engine")) { ... }
 * std::vector<float> output;
 * engine.infer(input_nchw.data(), input_nchw.size(), &output);
 * @endcode
 *
 * The input is always supplied as host FP32. If the TensorRT input binding is
 * FP16, the wrapper converts it before the H2D copy. The output is always
 * returned as host FP32; if the output binding is FP16, the wrapper converts it
 * after the D2H copy.
 */
class TrtEngine {
 public:
  /// @brief Construct an empty engine wrapper.
  TrtEngine();

  /// @brief Release the execution context, CUDA buffers and stream.
  ~TrtEngine();

  TrtEngine(const TrtEngine&) = delete;
  TrtEngine& operator=(const TrtEngine&) = delete;

  /**
   * @brief Deserialize an engine file and allocate CUDA resources.
   * @param engine_path Absolute path to a serialized TensorRT engine.
   * @return `true` on success; `false` if the file, runtime, engine, context
   *         or CUDA allocation fails.
   * @note The engine must have exactly one input and one output binding with
   *       linear FP32/FP16 data.
   */
  bool load(const std::string& engine_path);

  /**
   * @brief Run one synchronous inference.
   * @param input Host FP32 input tensor in NCHW layout.
   * @param input_elements Number of elements in @p input; must match the
   *        engine input binding.
   * @param output Destination vector; resized to the output binding size.
   * @return `true` on success; `false` on shape mismatch or CUDA/TensorRT
   *         failure.
   */
  bool infer(const float* input, size_t input_elements,
             std::vector<float>* output);

  /// @return `true` when the execution context and CUDA stream are ready.
  bool ok() const { return stream_ != nullptr; }

  /// @return Input binding dimensions, e.g. `{1, 3, 640, 640}`.
  const std::vector<int>& input_dims() const { return input_dims_; }

  /// @return Output binding dimensions, e.g. `{1, 25200, 85}`.
  const std::vector<int>& output_dims() const { return output_dims_; }

 private:
  /**
   * @brief Compute the element count of a TensorRT dimensions object.
   * @param dims TensorRT dimensions.
   * @return Product of all dimensions, or 0 if a dimension is unresolved.
   */
  static size_t volume(const nvinfer1::Dims& dims);

  /// @brief Release all TensorRT and CUDA resources and reset state.
  void release();

  TrtLogger logger_;                         ///< TensorRT logger instance.
  nvinfer1::IRuntime* runtime_ = nullptr;    ///< TensorRT runtime.
  nvinfer1::ICudaEngine* engine_ = nullptr;  ///< Deserialized engine.
  nvinfer1::IExecutionContext* context_ =
      nullptr;                               ///< Per-engine execution context.

  int input_index_ = -1;                     ///< Input binding index.
  int output_index_ = -1;                    ///< Output binding index.
  std::vector<int> input_dims_;              ///< Resolved input dimensions.
  std::vector<int> output_dims_;             ///< Resolved output dimensions.
  size_t input_elements_ = 0;                ///< Input element count.
  size_t output_elements_ = 0;               ///< Output element count.

  void* device_input_ = nullptr;             ///< Device input buffer.
  void* device_output_ = nullptr;            ///< Device output buffer.
  size_t input_bytes_ = 0;                   ///< Input buffer size in bytes.
  size_t output_bytes_ = 0;                  ///< Output buffer size in bytes.
  std::vector<__half> host_input_half_;      ///< FP16 host input staging.
  std::vector<__half> host_output_half_;     ///< FP16 host output staging.
  std::vector<float> host_output_;           ///< FP32 host output staging.
  cudaStream_t stream_ = nullptr;            ///< CUDA stream for copies/inference.
};

}  // namespace astra_rgbd_perception