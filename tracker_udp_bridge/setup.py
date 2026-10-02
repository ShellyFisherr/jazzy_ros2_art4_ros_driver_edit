from setuptools import find_packages, setup

package_name = 'tracker_udp_bridge'

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
    maintainer='hamgist',
    maintainer_email='d.f.j.krijnen@student.tue.nl',
    description='UDP bridge for steamVR tracker data to ROS2 topics',
    license='MIT licence',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [ 'udp_receiver = tracker_udp_bridge.udp_receiver:main'
        ],
    },
)
