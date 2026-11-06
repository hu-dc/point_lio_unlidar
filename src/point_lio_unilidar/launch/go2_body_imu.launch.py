#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Point-LIO for Unitree Go2 —— 用【机身 IMU】建图（推荐方案）。

与 go2_l1.launch.py 的区别：
  - 先起 lowstate_to_imu，把 /lowstate(500 Hz) 转成 /go2/imu
  - Point-LIO 读 config/go2_body_imu.yaml（imu_topic=/go2/imu，带机身->雷达外参）
  - use_imu_as_input=true（输入模型），实测比默认的 false 稳
  - 内部把 use_imu_as_input 固定为 true；如需改，直接编辑本文件

用法：
  # 在线（连着狗）
  ros2 launch point_lio_unilidar go2_body_imu.launch.py rviz:=true

  # 需要保存 PCD 地图（Ctrl-C 退出时写盘）
  ros2 launch point_lio_unilidar go2_body_imu.launch.py rviz:=true pcd_save_en:=true
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
    default_config = os.path.join(pkg_share, 'config', 'go2_body_imu.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz_cfg', 'loam_livox.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    config_file = LaunchConfiguration('config_file')
    pcd_save_en = LaunchConfiguration('pcd_save_en')
    odom_header_frame_id = LaunchConfiguration('odom_header_frame_id')
    check_satu = LaunchConfiguration('check_satu')
    input_topic = LaunchConfiguration('input_topic')
    imu_topic_out = LaunchConfiguration('imu_topic_out')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='离线 rosbag 回放时设 true（配合 ros2 bag play --clock）')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false', description='是否启动 rviz2')
    declare_config = DeclareLaunchArgument(
        'config_file', default_value=default_config, description='参数文件路径')
    declare_pcd_save = DeclareLaunchArgument(
        'pcd_save_en', default_value='false', description='是否保存 PCD 地图')
    declare_odom_frame = DeclareLaunchArgument(
        'odom_header_frame_id', default_value='camera_init',
        description='世界系名字（TF 父坐标系）')
    declare_check_satu = DeclareLaunchArgument(
        'check_satu', default_value='true',
        description='IMU 饱和检测：true 会丢弃超阈值的 IMU 样本')
    declare_input_topic = DeclareLaunchArgument(
        'input_topic', default_value='/lowstate',
        description='机身 IMU 来源话题（unitree_go/LowState）')
    declare_imu_out = DeclareLaunchArgument(
        'imu_topic_out', default_value='/go2/imu',
        description='桥接后发布的 sensor_msgs/Imu 话题（需与 config 里 imu_topic 一致）')

    # 1) /lowstate -> /go2/imu
    bridge = Node(
        package='point_lio_unilidar',
        executable='lowstate_to_imu.py',
        name='lowstate_to_imu',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'input_topic': input_topic,
            'out_topic': imu_topic_out,
        }],
    )

    # 2) Point-LIO
    laser_mapping = Node(
        package='point_lio_unilidar',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'odom_header_frame_id': odom_header_frame_id,
                'use_imu_as_input': True,       # 机身 IMU 用输入模型更稳
                'prop_at_freq_of_imu': True,
                'check_satu': ParameterValue(check_satu, value_type=bool),
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(rviz),
        prefix='nice',
    )

    return LaunchDescription([
        declare_use_sim_time, declare_rviz, declare_config, declare_pcd_save,
        declare_odom_frame, declare_check_satu, declare_input_topic, declare_imu_out,
        bridge,
        laser_mapping,
        rviz_node,
    ])
