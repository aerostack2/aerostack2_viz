"""
as2_viz.launch.py
"""

# Copyright 2022 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__authors__ = "Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2022 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Publish rosbag path for visualization"""

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Visualization markers
    viz = Node(
        package='as2_viz',
        executable='viz_rosbag_path',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            {'namespace': LaunchConfiguration('namespace')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'publish_rate': LaunchConfiguration('publish_rate')},
            {'topic_name': LaunchConfiguration('topic_name')},
            {'rosbag_path': LaunchConfiguration('rosbag_path')},
            {'marker.topic_name': LaunchConfiguration('marker.topic_name')},
            {'marker.color.r': LaunchConfiguration('marker.color.r')},
            {'marker.color.g': LaunchConfiguration('marker.color.g')},
            {'marker.color.b': LaunchConfiguration('marker.color.b')},
            {'marker.scale.x': LaunchConfiguration('marker.scale.x')},
            {'marker.scale.y': LaunchConfiguration('marker.scale.y')},
            {'marker.scale.z': LaunchConfiguration('marker.scale.z')},
        ]
    )

    default_rviz_config = os.path.join(get_package_share_directory('as2_viz'),
                                       'config', 'as2_default.rviz')

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz.'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config,
                              description='RViz configuration file.'),
        DeclareLaunchArgument('namespace', default_value='viz',
                              description='Node and topics namespace.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time'),
        DeclareLaunchArgument('publish_rate', default_value="1.0",
                              description='Marker topic publish rate.'),
        DeclareLaunchArgument('topic_name',
                              description='Topic name in the rosbag file.'),
        DeclareLaunchArgument('rosbag_path',
                              description='Rosbag db3 file path.'),
        DeclareLaunchArgument('marker.topic_name', default_value="",
                              description='Marker publisher topic name.'),
        DeclareLaunchArgument('marker.color.r', default_value="0.0",
                              description='Color red component for marker.'),
        DeclareLaunchArgument('marker.color.g', default_value="1.0",
                              description='Color green component for marker.'),
        DeclareLaunchArgument('marker.color.b', default_value="0.0",
                              description='Color blue component for marker.'),
        DeclareLaunchArgument('marker.scale.x', default_value="0.1",
                              description='Scale x component for marker.'),
        DeclareLaunchArgument('marker.scale.y', default_value="0.1",
                              description='Scale y component for marker.'),
        DeclareLaunchArgument('marker.scale.z', default_value="0.1",
                              description='Scale z component for marker.'),
        rviz,
        viz
    ])
