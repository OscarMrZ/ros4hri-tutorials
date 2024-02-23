# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch.actions import DeclareLaunchArgument
from launch_pal.arg_utils import LaunchArgumentsBase
from ament_index_python.packages import get_package_share_directory
from dataclasses import dataclass
from launch_pal.composition_utils import generate_component_list
from launch_ros.actions import LoadComposableNodes, Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_pal.include_utils import include_scoped_launch_py_description


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):

    tracked_face: DeclareLaunchArgument = DeclareLaunchArgument(
        name="tracked_face",
        default_value="testface",
        description="The face we are tracking",
    )


def declare_actions(
    launch_description: LaunchDescription, launch_args: LaunchArguments
):
    expressive_eyes = include_scoped_launch_py_description(
        pkg_name="expressive_eyes",
        paths=["launch", "expressive_eyes_with_eyes_tf.launch.py"],
    )

    launch_description.add_action(expressive_eyes)

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        parameters=[],
        arguments=['0', '0.1', '0', '0', '0', '0', 'default_cam', 'sellion_link'],
    )

    launch_description.add_action(camera_tf)

    rqt_display = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_image_view',
        arguments=['-s', 'image_view', '--args', '/robot_face/image_raw/compressed'],
        output='screen',
    )

    launch_description.add_action(rqt_display)

    return


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
