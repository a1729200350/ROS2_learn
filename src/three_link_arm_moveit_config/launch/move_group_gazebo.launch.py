from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
def generate_launch_description():
  # 加载 MoveIt 配置，但不允许 MoveIt 自动切换控制器
  # JTC 已有独立的启动和激活流程   禁用自动管理选项
  moveit_config = (
    MoveItConfigsBuilder("three_link_arm",package_name="three_link_arm_moveit_config")
    .trajectory_execution(moveit_manage_controllers=False).to_moveit_configs()
  )
  # 只启动 move_group，不创建第二个 controller_manager
  move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[
      moveit_config.to_dict(),
      {
        # 与 Gazebo 使用相同的仿真时间
        "use_sim_time": True,
        # 第二阶段 允许MoveIt执行轨迹
        "allow_trajectory_execution": True,
        # 发布规划场景，供后续 RViz 使用
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
      },
    ],
  )
  return LaunchDescription([ move_group_node])