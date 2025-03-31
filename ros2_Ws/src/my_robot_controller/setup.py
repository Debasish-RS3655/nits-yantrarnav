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
            # "position_node = my_robot_controller.position_publisher:main",
            
            # "path_planner_node = my_robot_controller.path_planner:main",
            # "path_planner_server_node = my_robot_controller.path_planner_server:main",
            # "path_planner_simulation_node = my_robot_controller.path_planner_simulation:main",
            
            # "path_planner_simulation_server_node = my_robot_controller.path_planner_simulation_server:main"            
            
            "bridge_server = my_robot_controller.bridge_server:main",
            # "path_mover = my_robot_controller.path_mover:main",
            "path_planner = my_robot_controller.path_planner:main",
            "path_manual = my_robot_controller.path_manual:main",
            "mode_controller = my_robot_controller.mode_controller:main",
            "launch_checker = my_robot_controller.launch_monitor:main",
            "coordinate = my_robot_controller.coordinate:main",
            "hover = my_robot_controller.coordinate:main",
            "perpendicular_webcam = my_robot_controller.perpendicular_webcam:main",
            "height_publisher = my_robot_controller.height:main",

            # replace with Shwetangshu and Mimansa's modules  
            "flat_area = my_robot_controller.flat_area:main",
            # "boundary_mapper = my_robot_controller.boundary_mapper:main",
            "odom = my_robot_controller.odom:main",
            "boundary_mapper = my_robot_controller.boundary_mapper:main",
            
        ],
    },
)
