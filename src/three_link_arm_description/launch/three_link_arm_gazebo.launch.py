import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (PythonLaunchDescriptionSource,)
from launch_ros.actions import Node
def generate_launch_description():
  description_share = get_package_share_directory( "three_link_arm_description")
  gazebo_share = get_package_share_directory("ros_gz_sim")
  urdf_path = os.path.join( description_share, "urdf", "three_link_arm_gazebo.urdf",)
  with open(urdf_path, "r", encoding="utf-8") as file:
    robot_description = file.read()
  # 将 URDF 中的 ROS 包路径替换为绝对路径
  robot_description = robot_description.replace("$(find three_link_arm_description)", description_share,)
  # 启动 Gazebo 空世界
  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(gazebo_share,"launch", "gz_sim.launch.py",)),
    launch_arguments={"gz_args": "-r empty.sdf",}.items(),
  )
  # 发布机器人描述与 TF
  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[{ "robot_description": robot_description, "use_sim_time": True,}],
  )

  # 将机器人模型加载到 Gazebo
  spawn_robot = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=[ "-topic", "robot_description", "-name", "three_link_arm",],
    output="screen",
  )

  # 将 Gazebo 的仿真时钟桥接到 ROS2
  clock_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    output="screen",
  )

  return LaunchDescription([
    gazebo,
    clock_bridge,
    robot_state_publisher,
    spawn_robot,
  ])
