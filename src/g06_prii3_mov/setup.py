from setuptools import setup

package_name = 'mov6'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[ 
        ('share/' + package_name + '/launch', ['launch/launch_jetbot.py']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='disa',
    maintainer_email='disa@todo.todo',
    description='JetBot drawing node and service',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetbot_draw = mov6.jetbot_draw:main',
            'jetbot_service = mov6.jetbot_service:main',
        ],
    },
)
