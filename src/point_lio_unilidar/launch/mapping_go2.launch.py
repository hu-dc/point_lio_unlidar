from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock from bag --clock')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false', description='Launch rviz2')

    params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio_unilidar'), 'config', 'go2_l1.yaml'
        ]),
        {
            'use_sim_time': use_sim_time,
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
        }
    ]
    node = Node(
        package='point_lio_unilidar', executable='pointlio_mapping', name='laserMapping',
        output='screen', parameters=params)
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio_unilidar'), 'rviz_cfg', 'loam_livox.rviz'])],
        condition=IfCondition(rviz), prefix='nice')
    return LaunchDescription([declare_use_sim_time, declare_rviz, node, rviz_node])
