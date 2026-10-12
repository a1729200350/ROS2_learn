import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
def generate_launch_description():
  description_share = get_package_share_directory("three_link_arm_description")
  gazebo_share = get_package_share_directory("ros_gz_sim")

  # 独立的 JTC URDF；其中 gz_ros2_control 插件加载 JTC 专用 YAML。
  urdf_path = os.path.join(description_share, "urdf", "three_link_arm_jtc_gazebo.urdf")
  with open(urdf_path, "r", encoding="utf-8") as file:
    robot_description = file.read()
  robot_description = robot_description.replace("$(find three_link_arm_description)", description_share)

  gazebo = IncludeLaunchDescription( 
    PythonLaunchDescriptionSource( os.path.join(gazebo_share, "launch", "gz_sim.launch.py")),
    launch_arguments={"gz_args": "-r empty.sdf"}.items(),
  )

  clock_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    output="screen",
  )

  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[{"robot_description": robot_description, "use_sim_time": True}],
  )

  spawn_robot = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=["-topic", "robot_description", "-name", "three_link_arm"],
    output="screen",
  )

  joint_state_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "joint_state_broadcaster",
      "--controller-manager", "/controller_manager",
      "--controller-manager-timeout", "120",
    ],
    output="screen",
  )

  # 首次实验只加载并配置 JTC，禁止自动激活。
  jtc_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "joint_trajectory_controller",
      "--controller-manager", "/controller_manager",
      "--controller-manager-timeout", "120",
      "--inactive",
    ],
    output="screen",
  )

  return LaunchDescription([
    gazebo,
    clock_bridge,
    robot_state_publisher,
    # 保留原项目验证过的事件顺序；只有上一进程正常退出才启动下一步。
    RegisterEventHandler(OnProcessExit(target_action=spawn_robot,on_exit=lambda event, context: ([joint_state_spawner] if event.returncode == 0 else []),)),
    RegisterEventHandler(OnProcessExit(target_action=joint_state_spawner,on_exit=lambda event, context: ([jtc_spawner] if event.returncode == 0 else []),)),
    spawn_robot,
  ])
