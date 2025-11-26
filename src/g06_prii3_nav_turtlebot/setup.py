from setuptools import setup, find_packages
import os

package_name = 'g06_prii3_nav_turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Instalar el launch file
        (os.path.join('share', package_name, 'launch'), ['launch/cositas_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wormi',
    maintainer_email='wormi@todo.todo',
    description='Waypoint navigation TB3',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'cositas = g06_prii3_nav_turtlebot.cositas:main',
        ],
    },
)
