from glob import glob
from setuptools import setup

setup(
    name='amr_remote_bridge', version='0.2.0', packages=['amr_remote_bridge'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/amr_remote_bridge']),
        ('share/amr_remote_bridge', ['package.xml', 'README.md']),
        ('share/amr_remote_bridge/launch', glob('launch/*.py')),
        ('share/amr_remote_bridge/web', glob('web/console.*')),
    ],
    install_requires=['setuptools'], extras_require={'test': ['pytest']}, zip_safe=True,
    maintainer='acus03', maintainer_email='acus03@example.com', license='TODO',
    entry_points={'console_scripts': [
        'remote_client = amr_remote_bridge.agent:main',
        'central_server = amr_remote_bridge.server:main',
    ]},
)
