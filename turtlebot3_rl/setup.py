from setuptools import setup

package_name = 'turtlebot3_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stanley',
    maintainer_email='stanley@example.com',
    description='RL package for TurtleBot3 in ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train = turtlebot3_rl.train:main',
            'test = turtlebot3_rl.test:main',
        ],
    },
)

