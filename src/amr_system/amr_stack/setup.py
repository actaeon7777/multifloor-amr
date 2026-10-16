from glob import glob
from setuptools import setup

package_name = 'amr_stack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        *[('share/' + package_name + '/' + directory, glob(directory + '/*'))
          for directory in sorted(glob('maps/*'))],
    ],
    extras_require={'test': ['pytest']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='acus02',
    maintainer_email='acus02@example.com',
    description='Launch and configuration for the AMR.',
    license='TODO',
    entry_points={'console_scripts': [
        'joy_autoselect = amr_stack.joy_autoselect:main',
        'navigation_manager = amr_stack.navigation_manager:main',
        'navigation_startup = amr_stack.navigation_startup:main',
        'scan_body_filter = amr_stack.scan_body_filter:main',
    ]},
)
