from setuptools import setup
import os
from glob import glob

package_name = 'g06_prii3_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavlo',
    maintainer_email='pavlo@example.com',
    description='Movimiento de turtlesim grupo 6',
    license='',
    entry_points={
        'console_scripts': [
            'turtle_draw = g06_prii3_turtlesim.turtle_draw:main',
            'turtle_service = g06_prii3_turtlesim.turtle_service:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
)
