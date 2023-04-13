from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    jetson_clocks_fan = ExecuteProcess(
        cmd=['/usr/bin/env', 'expect', '-f', '-'],
        stdin_line='spawn sudo jetson_clocks --fan\nexpect "Password:"\nsend "YourPasswordHere\\r"\ninteract'
    )

    pwm = Node(
        package='sailbot',
        node_executable='pwm_controller',
        name='pwm'
    )

    control_system = Node(
        package='sailbot',
        node_executable='control_system',
        name='ctrl_sys'
    )

    trim_tab_comms = Node(
        package='sailbot',
        node_executable='trim_tab_comms',
        name='trim_tab'
    )

    debug_interface = Node(
        package='sailbot',
        node_executable='debug_interface',
        name='debug'
    )

    airmar_reader = Node(
        package='sailbot',
        node_executable='airmar_reader',
        name='airmar'
    )

    serial_rc_receiver = Node(
        package='sailbot',
        node_executable='serial_rc_receiver',
        name='rc_rcevr'
    )

    return LaunchDescription([
        jetson_clocks_fan,
        pwm,
        control_system,
        trim_tab_comms,
        debug_interface,
        airmar_reader,
        serial_rc_receiver
    ])
