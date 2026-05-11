import glob

from setuptools import find_packages, setup

package_name = 'racecar_neo_ros2_driver'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob.glob('config/*.yaml')),
        ('share/' + package_name + '/scripts',
            glob.glob('scripts/*.sh') + glob.glob('scripts/*.py')),
        ('share/' + package_name + '/services',
            glob.glob('scripts/*.service')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='chrisclai02@gmail.com',
    description='ROS2 driver for MIT RACECAR Neo v2',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gamepad_node = racecar_neo_ros2_driver.gamepad_node:main',
            'mux_node = racecar_neo_ros2_driver.mux_node:main',
            'throttle_node = racecar_neo_ros2_driver.throttle_node:main',
            'pwm_node = racecar_neo_ros2_driver.pwm_node:main',
        ],
    },
)
