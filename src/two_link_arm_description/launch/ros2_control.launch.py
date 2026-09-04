from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
#generate_launch_description() 函数用于生成 launch 文件的描述信息
def generate_launch_description():
    pkg_name = "two_link_arm_description"
    # 找到 URDF 文件的路径
    urdf_file = PathJoinSubstitution(
        [
            FindPackageShare(pkg_name),
            "urdf",
            "two_link_arm.urdf"
        ]
    )
    # 读取 URDF 文件内容
    robot_description = ParameterValue(
        Command(
            [
                "cat ",
                urdf_file
            ]
        ),
        value_type=str
    )
    # 找到 controllers.yaml 文件的路径
    controllers_file = PathJoinSubstitution(
        [
            FindPackageShare(pkg_name),
            "config",
            "controllers.yaml"
        ]
    )
    #返回启动列表 启动四个节点
    #1. robot_state_publisher 节点：用于发布机器人状态信息，包括关节状态和 TF 变换。
    #2. ros2_control_node 节点：用于启动 ros2_control 控制器管理器，读URDF，加载机器人描述和控制器配置。
    #3. joint_state_broadcaster 节点：读各关节硬件状态，用于广播关节状态信息。
    #4. joint_trajectory_controller 节点：接收关节轨迹消息，用于控制机器人关节的轨迹运动。
    return LaunchDescription([
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {
                    "robot_description":
                    robot_description
                }
            ],
            output="screen"
        ),
        # ros2_control controller manager
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {
                    "robot_description":
                    robot_description
                },
                controllers_file
            ],
            output="screen"
        ),
        # joint state broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster"
            ],
            output="screen"
        ),
        # trajectory controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_trajectory_controller"
            ],
            output="screen"
        ),
        # rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2"
        )
    ])
