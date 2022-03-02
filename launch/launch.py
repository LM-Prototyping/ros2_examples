from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

import os
import sys
import argparse

from ament_index_python.packages import get_package_share_directory

AVAILABLE_EXAMPLES = [
    "set_velocity", 
    "set_power",
    "set_power_limit",
    "set_velocity_limit",
    "set_position_and_reset_offset",
    "set_position_and_set_offset",
    "set_position",
]

def parse_launch_arguments():
    parser = argparse.ArgumentParser()
    
    parser.add_argument("command")
    parser.add_argument("package")
    parser.add_argument("file")
    parser.add_argument("arguments", nargs='*')

    return parser.parse_args()


def build_arguments_dict(arguments):
    arg_dict = {}

    for arg in arguments:
        argument_name, value = arg.split(":=")

        if value == None:
            print("Cannot read value of argument", argument_name, " Please use: <argument_name>:=<value>")
            sys.exit(1)
    
        arg_dict[argument_name] = value

    return arg_dict


def generate_launch_description(): 
    args = parse_launch_arguments()
    arguments_dict = build_arguments_dict(args.arguments)

    example_name = arguments_dict["example"]

    sys.argv.append("robot_name:=CompleteRobot")

    if not example_name in AVAILABLE_EXAMPLES:
        print("Example not available")
        print("Available examples:", AVAILABLE_EXAMPLES)
        sys.exit(1)

    webots_interface = get_package_share_directory("brickpi3_ros2")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(webots_interface, "launch", "launch.py")
                ),
                launch_arguments={
                    "robot_name": "CompleteRobot"
                }.items()
            ),
            Node(
                package="ros2_examples",
                executable=example_name,
                output="screen",
                emulate_tty=True
            )
        ]
    )