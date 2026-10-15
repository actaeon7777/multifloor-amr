from setuptools import setup

package_name = 'amr_can_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    extras_require={'test': ['pytest']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='acus02',
    maintainer_email='acus02@example.com',
    description='AMR body velocity commands over SocketCAN.',
    license='TODO',
    entry_points={'console_scripts': [
        'cmd_vel_to_can = amr_can_bridge.cmd_vel_to_can:main',
        'can_telemetry = amr_can_bridge.can_telemetry:main',
        'elevator_can_link = amr_can_bridge.elevator_can_link:main',
    ]},
)
