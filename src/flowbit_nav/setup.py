from setuptools import find_packages, setup

package_name = 'flowbit_nav'

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
    maintainer='jetson',
    maintainer_email='lali.taye@determtech.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flowbit_nav = flowbit_nav.flowbit_nav:main',
            'initialpose_pub  = flowbit_nav.initialpose_pub:main', 
            'goal_pose_pub = flowbit_nav.goal_pose_pub:main', 
            'destination_controller = flowbit_nav.destination_controller:main', 
            'robot_server = flowbit_nav.robot_server:main'
        ],
    },
)
