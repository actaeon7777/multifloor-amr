from setuptools import setup

package_name = 'amr_patrol'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='AMR maintainers',
    maintainer_email='amr@example.com',
    description='Fixed-route patrol and observation hooks on the current floor.',
    license='TODO',
    entry_points={'console_scripts': [
        'patrol_manager = amr_patrol.patrol_node:main',
        'motion_detector = amr_patrol.detector_node:main',
    ]},
)
