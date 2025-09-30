from setuptools import find_packages, setup

package_name = 'multi_sensor_validator'

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
    maintainer='Moamen Khadra',
    maintainer_email='momentadev@gmail.com',
    description='Simulation of two range sensors and processing their values using multiple ROS2 nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_sensor_node = multi_sensor_validator.ultrasonic_sensor_node:main',
            'infrared_sensor_node = multi_sensor_validator.infrared_sensor_node:main',
            'validator_node = multi_sensor_validator.validator_node:main',
            'logger_node = multi_sensor_validator.logger_node:main',
            'obstacle_server_node = multi_sensor_validator.obstacle_server_node:main',
            'navigation_client_node = multi_sensor_validator.navigation_client_node:main'
        ],
    },
)
