from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dev = "/dev/ttyACM0"
    baud = 38400
    address = 128
    max_speed = 1.0
    ticks_per_meter = 2495.0
    base_width = 0.421
    run_diag = True
    invert_motor_direction = False
    flip_left_and_right_motors = False

    ld = LaunchDescription()

    roboclaw_node = Node(
        package='roboclaw_node',
        executable='robo_node',
        name='roboclaw_node',
        parameters=[{
            "dev": dev,
            "baud": baud,
            "address": address,
            "max_speed": max_speed,
            "ticks_per_meter": ticks_per_meter,
            "base_width": base_width,
            "run_diag": run_diag,
            "invert_motor_direction": invert_motor_direction,
            "flip_left_and_right_motors": flip_left_and_right_motors
        }]
    )
    ld.add_action(roboclaw_node)
    return ld
