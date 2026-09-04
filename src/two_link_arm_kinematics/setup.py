from setuptools import find_packages, setup

package_name = 'two_link_arm_kinematics'

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
          'kinematics_monitor = two_link_arm_kinematics.kinematics_monitor:main',
          'trajectory_monitor = two_link_arm_kinematics.trajectory_monitor:main',
          'trajectory_generator = two_link_arm_kinematics.trajectory_generator:main',
        ],
    },
)
