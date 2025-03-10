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
    maintainer='rahul',
    maintainer_email='rahul@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "position_node = my_robot_controller.position_publisher:main",
            "path_planner_node = my_robot_controller.path_planner:main",
            "path_planner_server_node = my_robot_controller.path_planner_server:main",
            "path_planner_simulation_node = my_robot_controller.path_planner_simulation:main",
            "path_planner_simulation_server_node = my_robot_controller.path_planner_simulation_server:main"
        ],
    },
)
