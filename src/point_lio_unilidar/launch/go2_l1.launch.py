#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Point-LIO (ROS2) for Unitree Go2 —— 只订阅话题，不启动任何雷达驱动。

用法：
  # 在线（连着狗，Go2 自己发布 /utlidar/cloud、/utlidar/imu）
  ros2 launch point_lio_unilidar go2_l1.launch.py

  # 离线回放（ros2 bag play --clock 时）
  ros2 launch point_lio_unilidar go2_l1.launch.py use_sim_time:=true rviz:=true

  # 保存地图（PCD 写到 <包目录>/PCD/scans.pcd）
  ros2 launch point_lio_unilidar go2_l1.launch.py pcd_save_en:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('point_lio_unilidar')
    default_config = os.path.join(pkg_share, 'config', 'go2_l1.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz_cfg', 'loam_livox.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    config_file = LaunchConfiguration('config_file')
    pcd_save_en = LaunchConfiguration('pcd_save_en')
    odom_header_frame_id = LaunchConfiguration('odom_header_frame_id')
    check_satu = LaunchConfiguration('check_satu')
    use_imu_as_input = LaunchConfiguration('use_imu_as_input')
    prop_at_freq_of_imu = LaunchConfiguration('prop_at_freq_of_imu')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='离线 rosbag 回放时设 true（配合 ros2 bag play --clock）')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false', description='是否启动 rviz2')
    declare_config = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='参数文件路径')
    declare_pcd_save = DeclareLaunchArgument(
        'pcd_save_en', default_value='false', description='是否保存 PCD 地图')
    declare_check_satu = DeclareLaunchArgument(
        'check_satu', default_value='true',
        description='IMU 饱和检测：true 会丢弃 |acc|/|gyro| 超阈值的 IMU 样本（激烈运动时建议 false）')
    declare_use_imu_as_input = DeclareLaunchArgument(
        'use_imu_as_input', default_value='false',
        description='false: IMU 作为量测（Point-LIO 默认）；true: IMU 作为输入（FAST-LIO 风格）')
    declare_prop_at_freq = DeclareLaunchArgument(
        'prop_at_freq_of_imu', default_value='true',
        description='是否按 IMU 频率做状态传播')
    declare_odom_frame = DeclareLaunchArgument(
        'odom_header_frame_id', default_value='camera_init',
        description='世界系名字（TF 父坐标系）')

    params = [
        config_file,
        {
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'odom_header_frame_id': odom_header_frame_id,
            # --- Point-LIO 核心开关（与官方 launch 保持一致）---
            'use_imu_as_input': ParameterValue(use_imu_as_input, value_type=bool),
            'prop_at_freq_of_imu': ParameterValue(prop_at_freq_of_imu, value_type=bool),
            'check_satu': ParameterValue(check_satu, value_type=bool),
            'init_map_size': 10,
            'point_filter_num': 1,          # Go2 点云很稀，不抽点
            'space_down_sample': True,
            'filter_size_surf': 0.1,
            'filter_size_map': 0.1,
            'cube_side_length': 1000.0,
            'runtime_pos_log_enable': False,
            # --- 地图保存 ---
            'pcd_save.pcd_save_en': ParameterValue(pcd_save_en, value_type=bool),
            'pcd_save.interval': -1,
        },
    ]

    laser_mapping = Node(
        package='point_lio_unilidar',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=params,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(rviz),
        prefix='nice',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_config,
        declare_pcd_save,
        declare_check_satu,
        declare_use_imu_as_input,
        declare_prop_at_freq,
        declare_odom_frame,
        laser_mapping,
        rviz_node,
    ])
