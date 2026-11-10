# -----------------------------------------------------------------------------
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Basalt VIO on the front ZED X of the AMIAD rig (Isaac Sim).
# Default: ros camera mode, the sim publishes /zed_front/{left,right}/image_rect_color (rgb8,
# converted to gray by the frontend).
# zed camera mode: run the ZED wrapper with gray images and pass the topics:
#   ros2 launch amiad_description zed_sim_cameras.launch.py cameras:=front \
#     depth_mode:=NONE param_overrides:='video.publish_gray:=true'
#   ros2 launch basalt_ros vio_composed_zedx_front.launch.py \
#     left_image:=/zed_front/zed_node/left/gray/rect/image \
#     right_image:=/zed_front/zed_node/right/gray/rect/image
#

import launch
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def pkg_file(fname):
    return PathJoinSubstitution(
        [FindPackageShare('basalt_ros'), 'config', fname])


def launch_setup(context, *args, **kwargs):
    """Create container."""
    use_sim_time = LaunchConfig('use_sim_time').perform(context).lower() == 'true'
    common_parameters = \
        {'calibration_file': pkg_file(LaunchConfig('calib_file')),
         'vio_config_file': pkg_file(LaunchConfig('config_file')),
         'use_sim_time': use_sim_time}
    frontend_parameters = common_parameters
    backend_parameters = {**common_parameters,
                          **{'has_split_accel_and_gyro_topics': False,
                             'world_frame_id': LaunchConfig('world_frame'),
                             'child_frame_id': 'pulse80_imu_link',
                             'base_frame_id': LaunchConfig('base_frame'),
                             'publish_tf': LaunchConfig('publish_tf')}}

    frontend_remappings = [
        ('left_image', LaunchConfig('left_image')),
        ('right_image', LaunchConfig('right_image'))]
    backend_remappings = [
        ('optical_flow', LaunchConfig('flow_topic')),
        ('imu', LaunchConfig('imu_topic'))]

    container = ComposableNodeContainer(
            name='basalt_vio_container',
            namespace='basalt',
            package='rclcpp_components',
            executable='component_container',
            parameters=[{'use_sim_time': use_sim_time}],
            composable_node_descriptions=[
                ComposableNode(
                    package='basalt_ros',
                    plugin='basalt_ros::VIOFrontEndNode',
                    name='vio_frontend',
                    namespace='basalt',
                    parameters=[frontend_parameters],
                    remappings=frontend_remappings,
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='basalt_ros',
                    plugin='basalt_ros::VIOBackEndNode',
                    name='vio_backend',
                    namespace='basalt',
                    parameters=[backend_parameters],
                    remappings=backend_remappings,
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen')
    return [container]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription([
        LaunchArg('left_image',
                  default_value=['/zed_front/left/image_rect_color'],
                  description='left rectified image (mono8 or rgb8/bgr8/rgba8/bgra8)'),
        LaunchArg('right_image',
                  default_value=['/zed_front/right/image_rect_color'],
                  description='right rectified image (mono8 or rgb8/bgr8/rgba8/bgra8)'),
        LaunchArg('calib_file', default_value=['zedx_front_calib.json'],
                  description='camera calibration file'),
        LaunchArg('config_file', default_value=['zedx_vio_config.json'],
                  description='vio config file'),
        LaunchArg('flow_topic',
                  default_value=['optical_flow'],
                  description='optical flow topic'),
        LaunchArg('imu_topic',
                  default_value=['/pulse80/imu'],
                  description='combined accel/gyro topic (Isaac Sim, zed_front_camera_center)'),
        LaunchArg('base_frame', default_value=['base_link'],
                  description='frame published as child of world_frame '
                  '(empty: the IMU frame)'),
        LaunchArg('publish_tf', default_value=['true'],
                  description='broadcast world_frame -> base_frame on TF'),
        LaunchArg('world_frame',
                  default_value=['basalt_odom'],
                  description='frame of the published VIO odometry'),
        LaunchArg('use_sim_time', default_value=['true'],
                  description='use the Isaac Sim /clock'),
        OpaqueFunction(function=launch_setup)
        ])
