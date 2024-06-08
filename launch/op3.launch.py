import os
import xacro
import launch_ros

from launch import LaunchDescription
from launch.substitutions import Command
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  project_path = PathJoinSubstitution([
    FindPackageShare("op3_simulator"),
    "project",
    "op3.cnoid"
  ])

  rviz_config = PathJoinSubstitution([
    FindPackageShare("op3_simulator"),
    "rviz",
    "op3_simulator.rviz"
  ])

  robotis_op3_urdf_path= os.path.join(get_package_share_directory('op3_description'), 'urdf', 'robotis_op3.urdf.xacro')
  robot_description = xacro.process_file(robotis_op3_urdf_path, mappings={'name': 'robotis_op3'}).toxml()

  return LaunchDescription([
    DeclareLaunchArgument("use_sim_time", default_value="true"),
    DeclareLaunchArgument("rviz", default_value="false"),
    DeclareLaunchArgument("gui", default_value="false"),
    DeclareLaunchArgument("project_path", default_value="project_path"),

    Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        remappings=[("/joint_states", "/robotis_op3/joint_states")],
        parameters=[{"robot_description": robot_description}, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    ),
    Node(
      package="choreonoid_ros",
      executable="choreonoid",
      arguments=["--start-simulation", project_path]
    ),
    Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      arguments=["-d", rviz_config],
      condition=IfCondition(LaunchConfiguration("rviz")),
    ),
    Node(
      package='joint_state_publisher_gui',
      executable='joint_state_publisher_gui',
      condition=IfCondition(LaunchConfiguration('gui')),
      remappings=[('/joint_states', '/target_joint_states')]
    )
  ])
