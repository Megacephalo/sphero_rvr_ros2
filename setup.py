from setuptools import find_packages, setup

package_name = 'sphero_rvr_ros2'

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
    maintainer='charlyhuang',
    maintainer_email='charly.charlongo@gmail.com',
    description='My barebone ROS2 node to drive the Sphero RVR+',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rvrp_node = sphero_rvr_ros2.rvrp_node:main'
        ],
    },
)
