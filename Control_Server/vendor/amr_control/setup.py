from setuptools import setup

package_name = 'amr_control'

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
    description='Manual and autonomous AMR command selection.',
    license='TODO',
    entry_points={'console_scripts': [
        'control_mux = amr_control.control_mux:main',
        'time_health = amr_control.time_health:main',
    ]},
)
