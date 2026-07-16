from setuptools import find_packages, setup

package_name = 'custom_actions'

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
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fibonacci_action_server = custom_actions.fibonacci_action_server:main',
            'fibonacci_action_client = custom_actions.fibonacci_action_client:main',
            'imu_action_server = custom_actions.imu_action_server:main',
            'imu_action_client = custom_actions.imu_action_client:main',
        ],
    },
)
