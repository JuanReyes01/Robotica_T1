from setuptools import find_packages, setup

package_name = 'my_robot_controller'

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
    maintainer='juan',
    maintainer_email='juan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "1_turtle_bot_teleop = my_robot_controller.turtle_bot_teleop:main",
            "2_turtle_bot_interface = my_robot_controller.turtle_bot_interface:main",
            "3_turtle_bot_player = my_robot_controller.turtle_bot_player:main"
        ],
    },
)
