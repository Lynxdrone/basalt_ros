#!/usr/bin/env python3
"""Write a basalt calibration file for the ZEDs of the AMIAD rig.

The ZED wrapper publishes rectified images, so every camera is a pinhole without
distortion. The intrinsics are read from the rectified camera_info topics (or
estimated with --estimate), the camera poses in the IMU frame from the rig URDF
(amiad_description/urdf/amiad.urdf.xacro). The IMU is the one Isaac Sim publishes
on /zed_front/imu, at zed_front_camera_center (x forward, y left, z up).

Cameras are written in stereo pairs, left then right, in the order of --arms.

    python3 zed_calib_from_camera_info.py --arms front -o ../config/zedx_front_calib.json
    python3 zed_calib_from_camera_info.py --arms front,left,rear,right \\
        -o ../config/zedx_4stereo_calib.json
"""
import argparse
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def quaternion(r):
    """(qx, qy, qz, qw) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return x, y, z, w


class Urdf:
    """Fixed joint tree of a URDF: 4x4 pose of any link in any other."""

    def __init__(self, xml):
        self.parent = {}
        for joint in ET.fromstring(xml).iter('joint'):
            origin = joint.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0, 0, 0]
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0, 0, 0]
            t = np.eye(4)
            t[:3, :3] = rpy_matrix(*rpy)
            t[:3, 3] = xyz
            self.parent[joint.find('child').get('link')] = (joint.find('parent').get('link'), t)

    def root_pose(self, link):
        t = np.eye(4)
        while link in self.parent:
            link, t_parent_link = self.parent[link]
            t = t_parent_link @ t
        return t

    def pose(self, frame, link):
        return np.linalg.inv(self.root_pose(frame)) @ self.root_pose(link)


def load_urdf(path):
    if path:
        with open(path) as f:
            return Urdf(f.read())
    import xacro
    from ament_index_python.packages import get_package_share_directory
    xacro_file = os.path.join(get_package_share_directory('amiad_description'), 'urdf', 'amiad.urdf.xacro')
    return Urdf(xacro.process_file(xacro_file).toxml())


def camera_infos(arms):
    """Rectified camera_info of every camera, {(arm, side): CameraInfo}."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo

    rclpy.init()
    node = Node('zed_calib_from_camera_info')
    infos = {}
    for arm in arms:
        for side in ('left', 'right'):
            topic = f'/zed_{arm}/zed_node/{side}/gray/rect/image/camera_info'
            node.create_subscription(
                CameraInfo, topic, lambda msg, k=(arm, side): infos.setdefault(k, msg), 10)
            node.get_logger().info(f'waiting for {topic}')
    while rclpy.ok() and len(infos) < 2 * len(arms):
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    return {k: ((m.p[0], m.p[5], m.p[2], m.p[6]), (m.width, m.height)) for k, m in infos.items()}


def estimated_infos(arms, width, height, hfov_deg):
    """Ideal pinhole with the ZED X Wide field of view (isaac/run_sim.py)."""
    f = width / 2 / math.tan(math.radians(hfov_deg) / 2)
    return {(arm, side): ((f, f, width / 2, height / 2), (width, height))
            for arm in arms for side in ('left', 'right')}


def calibration(urdf, infos, arms, imu_frame, imu_rate):
    t_imu_cam, intrinsics, resolution = [], [], []
    for arm in arms:
        for side in ('left', 'right'):
            t = urdf.pose(imu_frame, f'zed_{arm}_{side}_camera_optical_frame')
            qx, qy, qz, qw = quaternion(t[:3, :3])
            px, py, pz = (round(float(v), 9) for v in t[:3, 3])
            t_imu_cam.append({'px': px, 'py': py, 'pz': pz,
                              'qx': round(qx, 12), 'qy': round(qy, 12),
                              'qz': round(qz, 12), 'qw': round(qw, 12)})
            (fx, fy, cx, cy), size = infos[(arm, side)]
            intrinsics.append({'camera_type': 'pinhole',
                               'intrinsics': {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy}})
            resolution.append(list(size))
            print(f'cam {len(intrinsics) - 1}: zed_{arm} {side}, p ({px:.4f}, {py:.4f}, {pz:.4f}), '
                  f'fx {fx:.3f}, {size[0]}x{size[1]}')
    return {'value0': {
        'T_imu_cam': t_imu_cam,
        'intrinsics': intrinsics,
        'resolution': resolution,
        'calib_accel_bias': [0.0] * 9,
        'calib_gyro_bias': [0.0] * 12,
        'imu_update_rate': imu_rate,
        # the simulated IMU is noise free: these only weight the IMU factors
        'accel_noise_std': [0.008, 0.008, 0.008],
        'gyro_noise_std': [0.002, 0.002, 0.002],
        'accel_bias_std': [0.01, 0.01, 0.01],
        'gyro_bias_std': [0.0007, 0.0007, 0.0007],
        'cam_time_offset_ns': 0,
        'vignette': [],
    }}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--arms', default='front', help='comma separated, e.g. front,left,rear,right')
    parser.add_argument('--imu-frame', default='zed_front_camera_center')
    parser.add_argument('--imu-rate', type=float, default=200.0)
    parser.add_argument('--urdf', default='', help='URDF file (default: amiad_description amiad.urdf.xacro)')
    parser.add_argument('--estimate', action='store_true',
                        help='ideal pinhole intrinsics instead of reading camera_info')
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--height', type=int, default=600)
    parser.add_argument('--hfov', type=float, default=110.0)
    parser.add_argument('-o', '--output', required=True)
    args = parser.parse_args()

    arms = [a.strip() for a in args.arms.split(',') if a.strip()]
    urdf = load_urdf(args.urdf)
    infos = (estimated_infos(arms, args.width, args.height, args.hfov) if args.estimate
             else camera_infos(arms))
    with open(args.output, 'w') as f:
        json.dump(calibration(urdf, infos, arms, args.imu_frame, args.imu_rate), f, indent=4)
        f.write('\n')
    print(f'wrote {args.output}')


if __name__ == '__main__':
    main()
