from setuptools import find_packages, setup

package_name = 'three_link_arm_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wkj',
    maintainer_email='a1729200350@gmail.com',
    description='Planar three-link arm kinematics and inverse-velocity experiments.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'kinematics_monitor = three_link_arm_kinematics.kinematics_monitor:main',
          'trajectory_monitor = three_link_arm_kinematics.trajectory_monitor:main',
          'trajectory_generator = three_link_arm_kinematics.trajectory_generator:main',
          'dynamics_monitor = three_link_arm_kinematics.dynamics_monitor:main',
          'joint_space_control_monitor = three_link_arm_kinematics.joint_space_control_monitor:main',
          'computed_torque_control_monitor = three_link_arm_kinematics.computed_torque_control_monitor:main',
          'joint_dynamics_simulator = three_link_arm_kinematics.joint_dynamics_simulator:main',
          'trajectory_tracking_analyzer = three_link_arm_kinematics.trajectory_tracking_analyzer:main',
        ],
    },
)
