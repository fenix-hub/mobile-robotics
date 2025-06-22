from setuptools import setup

package_name = 'ackermann_control_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fenixhub',
    maintainer_email='fenixhub@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_bridge_node = ackermann_control_bridge.ackermann_bridge_node:main',
            'ackermann_odometry_node = ackermann_control_bridge.ackermann_odometry_node:main'
        ],
    },
)
