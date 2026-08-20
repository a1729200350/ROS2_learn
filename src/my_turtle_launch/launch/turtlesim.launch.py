from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot1_sim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtle_sim_1',
        namespace='robot1',

        parameters=[{
            'background_r': 255,
            'background_g': 100,
            'background_b': 100
        }],
        remappings=[
            ('turtle1/cmd_vel', 'cmd_vel'),
            ('turtle1/pose', 'pose')
        ]
    )
    robot2_sim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtle_sim_2',
        namespace='robot2',
        parameters=[{
            'background_r': 100,
            'background_g': 100,
            'background_b': 255
        }],
        remappings=[
            ('turtle1/cmd_vel', 'cmd_vel'),
            ('turtle1/pose', 'pose')
        ]
    )
    return LaunchDescription([
        robot1_sim_node,
        robot2_sim_node
    ])
