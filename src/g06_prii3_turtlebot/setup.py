from setuptools import setup
from glob import glob
import os

package_name = 'g06_prii3_turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Necesario para que ROS2 detecte el paquete
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Instalar package.xml
        ('share/' + package_name, ['package.xml']),

        # Instalar todos los launch files (*.py)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Instalar archivos .world
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavlo',
    maintainer_email='pabrodbal@gmail.com',
    description='Custom TurtleBot3 world and launch files for PRI',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
