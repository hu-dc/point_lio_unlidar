#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线用：先起 Point-LIO，再自动回放 Go2 的 rosbag。

用法：
  ros2 launch point_lio_unilidar go2_bag_replay.launch.py \
      bag_path:=$HOME/go2_bags/20261102_163913_static rviz:=true

会启动：
  1. point_lio_unilidar/pointlio_mapping（use_sim_time=true）
  2. ros2 bag play <bag_path> --clock（延时 2 s，只回放 cloud/imu 话题）
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
    default_config = os.path.join(pkg_share, 'config', 'go2_l1.yaml')
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

    laser_mapping = Node(
        package='point_lio_unilidar',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': True,
                'use_imu_as_input': False,
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
            '--topics', '/utlidar/cloud', '/utlidar/imu',
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(rviz), prefix='nice')

    return LaunchDescription([
        declare_bag, declare_rate, declare_rviz, declare_config, declare_pcd,
        laser_mapping,
        TimerAction(period=2.0, actions=[bag_play]),
        rviz_node,
    ])
