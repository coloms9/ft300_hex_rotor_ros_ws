#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    m = LaunchConfiguration('motor_thrust')
    ti = LaunchConfiguration('test_time')
    f = LaunchConfiguration('file_location')

    m_launch_arg = DeclareLaunchArgument(
        'motor_thrust',
        default_value = '0,0,0,0,0,0'
    )

    t_launch_arg = DeclareLaunchArgument(
        'test_time',
        default_value = '1.5'
    )

    f_launch_arg = DeclareLaunchArgument(
        'file_location',
        default_value='~/ur5_hex_copter_data_store/test_default'
    )

    start_motor_test = ExecuteProcess(
        cmd = [[
            'ros2 ',
            'run ',
            'px4_stuff ',
            'px4_motor_test7 '
            '-m ',
            m,
            ' -t ',
            ti
        ]],
        shell=True
    )

    start_ros_bag = ExecuteProcess(
        cmd = [[
            'ros2 ',
            'bag ',
            'record ',
            '-o ',
            f,
            ' /ft300_data/raw /ft300_data/exp_filtered /ft300_data/lowpass_filtered '
            'fmu/in/vehicle_command ',
            '--qos-profile-overrides-path /home/jj/qos_settings/px4_reliability_override.yaml',
        ]],
        shell=True
    )

    return LaunchDescription([
        
        m_launch_arg,
        t_launch_arg,
        f_launch_arg,
        start_ros_bag,
        start_motor_test,

    ])