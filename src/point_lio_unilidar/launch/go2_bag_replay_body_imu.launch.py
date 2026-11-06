#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线用（推荐方案）：机身 IMU + Point-LIO，自动回放 Go2 rosbag。

用法：
  ros2 launch point_lio_unlidar go2_bag_replay_body_imu.launch.py \
      bag_path:=$HOME/go2_bags/20261105_095434_walk_308_4 rviz:=true

会启动：
  1. lowstate_to_imu     ：/lowstate -> /go2/imu
  2. pointlio_mapping    ：config/go2_body_imu.yaml，use_sim_time=true
  3. ros2 bag play       ：延迟 2 s 回放 /utlidar/cloud /utlidar/imu /lowstate --clock
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('point_lio_unilidar')
    default_config = os.path.join(pkg_share, 'config', 'go2_body_imu.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz_cfg', 'loam_livox.rviz')

    bag_path = LaunchConfiguration('bag_path')
    rate = LaunchConfiguration('rate')
    rviz = LaunchConfiguration('rviz')
    config_file = LaunchConfiguration('config_file')
    pcd_save_en = LaunchConfiguration('pcd_save_en')

    declare_bag = DeclareLaunchArgument('bag_path', description='rosbag2 目录')
    declare_rate = DeclareLaunchArgument('rate', default_value='1.0')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false')
    declare_config = DeclareLaunchArgument('config_file', default_value=default_config)
    declare_pcd = DeclareLaunchArgument(
        'pcd_save_en', default_value='false', description='是否保存 PCD 地图')

    bridge = Node(
        package='point_lio_unilidar',
        executable='lowstate_to_imu.py',
        name='lowstate_to_imu',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/lowstate',
            'out_topic': '/go2/imu',
        }],
    )

    laser_mapping = Node(
        package='point_lio_unilidar',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': True,
                'use_imu_as_input': True,
                'prop_at_freq_of_imu': True,
                'check_satu': True,
                'init_map_size': 10,
                'point_filter_num': 1,
                'space_down_sample': True,
                'filter_size_surf': 0.1,
                'filter_size_map': 0.1,
                'cube_side_length': 1000.0,
                'runtime_pos_log_enable': False,
                'pcd_save.pcd_save_en': ParameterValue(pcd_save_en, value_type=bool),
                'pcd_save.interval': -1,
            },
        ],
    )

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock', '--rate', rate,
            '--topics', '/utlidar/cloud', '/utlidar/imu', '/lowstate',
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(rviz), prefix='nice')

    return LaunchDescription([
        declare_bag, declare_rate, declare_rviz, declare_config, declare_pcd,
        bridge,
        laser_mapping,
        TimerAction(period=3.0, actions=[bag_play]),
        rviz_node,
    ])
