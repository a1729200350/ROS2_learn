import os
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import ( IncludeLaunchDescription, RegisterEventHandler, )
from launch.launch_description_sources import (PythonLaunchDescriptionSource,)
from launch_ros.actions import Node
def generate_launch_description():
  description_share = get_package_share_directory( "three_link_arm_description")
  gazebo_share = get_package_share_directory("ros_gz_sim")
  urdf_path = os.path.join( description_share, "urdf", "three_link_arm_gazebo.urdf",)
  osc_params_path = os.path.join(description_share,"config","osc_params.yaml",)
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

    # 自动启动关节状态广播器
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

  # 加载力矩控制器，但暂时不激活
  effort_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "effort_controller",
      "--controller-manager", "/controller_manager",
      "--controller-manager-timeout", "120",
      "--inactive",
    ],
    output="screen",
  )

  # 启动操作空间控制器
  osc_node = Node(
    package="three_link_arm_kinematics",
    executable="operational_space_control_monitor",
    output="screen",
    parameters=[
      osc_params_path,
      {"use_sim_time": True}
    ],
    remappings=[
      (
        "/joint_effort_command",
        "/effort_controller/commands",
      ),
    ],
  )
  auto_activator_node = Node(
    package="three_link_arm_kinematics",
    executable="osc_auto_activator",
    output="screen",
  )

  # 独立 OSC 故障监控程序
  osc_watchdog = ExecuteProcess(
    cmd=[
      "/usr/bin/python3",
      "-u",
      os.path.expanduser("~/ros2_ws/scripts/osc_controller_state_watch.py"),
    ],
    output="screen",
  )

  return LaunchDescription([
    gazebo,
    clock_bridge,
    robot_state_publisher,
    osc_watchdog,
    # 注册启动顺序
    # 模型生成成功后，启动关节状态广播器
    RegisterEventHandler(OnProcessExit(target_action=spawn_robot,on_exit=lambda event, context:([joint_state_spawner] if event.returncode == 0 else []),)),
    # 状态广播器启动成功后，加载力矩控制器
    RegisterEventHandler(OnProcessExit(target_action=joint_state_spawner,on_exit=lambda event, context:([effort_spawner] if event.returncode == 0 else []),)),
    # 力矩控制器加载成功后，启动 OSC 和自动激活节点
    RegisterEventHandler(OnProcessExit(target_action=effort_spawner,on_exit=lambda event, context: ([osc_node,auto_activator_node] if event.returncode == 0 else []),)),
    spawn_robot,
  ])
