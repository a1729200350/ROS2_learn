import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    # 找到当前功能包安装后的路径
    package_path = get_package_share_directory('two_link_arm_description')
    # 找到 URDF 文件的路径
    urdf_path = os.path.join(package_path, 'urdf', 'two_link_arm.urdf')
    # 读取 URDF 文件内容
    with open(urdf_path, 'r') as file:
        robot_description = file.read()
    # 根据 URDF 和 joint_states 发布 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    # # 提供 joint1、joint2 的滑块
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     parameters=[{'rate': 600}]
    # )
    # 启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
        rviz_node
    ])
