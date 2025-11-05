from setuptools import setup

package_name = 'g06_prii3_move_turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', [
             'launch/launch_turtlebot.py',
             'launch/launch_turtlebot_lidar.py',
             'launch/launch_turtlebot_lidar_avoidance.py',
        ]),
        
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavlo',
    maintainer_email='pabrodbal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_draw = g06_prii3_move_turtlebot.turtlebot_draw:main',
            'turtlebot_draw_lidar = g06_prii3_move_turtlebot.turtlebot_draw_lidar:main',
            'turtlebot_draw_lidar_avoidance = g06_prii3_move_turtlebot.turtlebot_draw_lidar_avoidance:main',
            'turtlebot_service = g06_prii3_move_turtlebot.turtlebot_service:main',
        ],
    },
)
