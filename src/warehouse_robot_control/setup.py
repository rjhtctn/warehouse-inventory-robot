from setuptools import find_packages, setup

setup(
    name='warehouse_robot_control',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/warehouse_robot_control']),
        ('share/warehouse_robot_control', ['package.xml']),
        ('share/warehouse_robot_control/config', ['config/control.yaml', 'config/inspection.yaml',
                                                 'config/patrol.yaml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Yusuf Emre Gözcü',
    maintainer_email='yusuf_emre_gzc@localhost.localdomain',
    description='Sensor monitoring and reactive control for a warehouse rover.',
    license='MIT',
    entry_points={'console_scripts': [
        'obstacle_avoidance = warehouse_robot_control.obstacle_avoidance:main',
        'sensor_monitor = warehouse_robot_control.sensor_monitor:main',
        'motion_guard = warehouse_robot_control.motion_guard:main',
        'inventory_mission = warehouse_robot_control.inventory_mission:main',
        'inventory_patrol = warehouse_robot_control.inventory_patrol:main',
    ]},
)
