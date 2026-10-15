from setuptools import setup

package_name = 'amr_elevator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    extras_require={'test': ['pytest']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AMR maintainers',
    maintainer_email='amr@example.com',
    description='Elevator door sensing, floor/zone estimation and multi-floor missions.',
    license='TODO',
    entry_points={'console_scripts': [
        'elevator_monitor = amr_elevator.monitor_node:main',
        'floor_mission_manager = amr_elevator.mission_node:main',
    ]},
)
