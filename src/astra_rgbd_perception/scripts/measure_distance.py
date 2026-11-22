#!/usr/bin/env python2
## @file measure_distance.py
## @brief Measure the 3D Z of a detected object for tape-measure validation.
## @details
## Subscribes to `/perception/objects_3d`, selects the valid detection of a
## requested class closest to the image center, and prints median Z/X/Y and
## confidence over a sampling window. Compare `median_z` with the physical
## distance measured from the camera front to the object front.
##
## Usage (camera/system running):
##
##     python2 measure_distance.py --class bottle --duration 5
##     python2 measure_distance.py --class cup --duration 5 --show-all
##
## @copyright MIT License, 2026.
"""Measure the 3D Z of a detected object for the tape-measure accuracy test.

Put the object at a measured distance from the camera, keep it still, run the
script, and compare "median_z" with the tape measure.
"""

import argparse
import sys

import numpy as np
import rospy

from astra_rgbd_perception.msg import Object3DArray


class Collector(object):
    """@brief Collect valid 3D detections for one class and summarize them."""

    def __init__(self, class_name, min_confidence):
        """@brief Initialize the collector.

        @param class_name Target class name (e.g. `bottle`, `cup`).
        @param min_confidence Minimum detection confidence.
        """
        self.class_name = class_name
        self.min_confidence = min_confidence
        self.samples = []
        self.class_counts = {}

    def callback(self, msg):
        """@brief Process one `Object3DArray` message.

        @param msg Object3DArray from `/perception/objects_3d`.
        """
        best = None
        best_dist = 1e9
        for obj in msg.objects:
            self.class_counts[obj.class_name] = \
                self.class_counts.get(obj.class_name, 0) + 1
            if obj.class_name != self.class_name:
                continue
            if not obj.depth_valid or obj.confidence < self.min_confidence:
                continue
            cx = 0.5 * (obj.xmin + obj.xmax)
            cy = 0.5 * (obj.ymin + obj.ymax)
            dist = ((cx - 320.0) ** 2 + (cy - 240.0) ** 2) ** 0.5
            if dist < best_dist:
                best_dist = dist
                best = obj
        if best is not None:
            self.samples.append({
                'z': best.depth_m,
                'x': best.position.x,
                'y': best.position.y,
                'conf': best.confidence,
                'bbox': (best.xmin, best.ymin, best.xmax, best.ymax),
            })


def main():
    """@brief Parse arguments, collect samples and print the measurement.

    @return 0 on success, 1 when no valid target detection was received.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--class', dest='class_name', default='bottle')
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--topic', default='/perception/objects_3d')
    parser.add_argument('--min-confidence', type=float, default=0.25)
    parser.add_argument('--show-all', action='store_true',
                        help='print all detected class names')
    args = parser.parse_args()

    rospy.init_node('measure_distance', anonymous=True)
    collector = Collector(args.class_name, args.min_confidence)
    rospy.Subscriber(args.topic, Object3DArray, collector.callback,
                     queue_size=5)
    print('[measure] class=%s duration=%.1fs; keep the object still...' %
          (args.class_name, args.duration))
    sys.stdout.flush()
    rospy.Timer(rospy.Duration(args.duration),
                lambda e: rospy.signal_shutdown('done'))
    rospy.spin()

    if args.show_all or not collector.samples:
        counts = sorted(collector.class_counts.items(), key=lambda kv: -kv[1])
        print('[measure] detected classes: %s' %
              ', '.join('%s:%d' % (k, v) for k, v in counts[:15]))

    if not collector.samples:
        print('[measure] RESULT: no valid "%s" detection with depth. '
              'Try another --class or move/scale the object.' % args.class_name)
        return 1

    zs = np.array([s['z'] for s in collector.samples], dtype=np.float64)
    xs = np.array([s['x'] for s in collector.samples], dtype=np.float64)
    ys = np.array([s['y'] for s in collector.samples], dtype=np.float64)
    confs = np.array([s['conf'] for s in collector.samples], dtype=np.float64)
    print('[measure] frames=%d' % len(zs))
    print('[measure] median_z=%.3f m mean_z=%.3f m p5=%.3f p95=%.3f '
          'min=%.3f max=%.3f' % (np.median(zs), zs.mean(),
                                 np.percentile(zs, 5), np.percentile(zs, 95),
                                 zs.min(), zs.max()))
    print('[measure] mean_x=%.3f m mean_y=%.3f m mean_conf=%.3f' %
          (xs.mean(), ys.mean(), confs.mean()))
    print('[measure] median_x=%.3f m median_y=%.3f m' %
          (np.median(xs), np.median(ys)))
    print('[measure] last bbox=%s' % (collector.samples[-1]['bbox'],))
    return 0


if __name__ == '__main__':
    sys.exit(main())