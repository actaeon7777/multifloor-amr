from glob import glob
from setuptools import setup

setup(
    name='amr_web_ui', version='0.1.0', packages=['amr_web_ui'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/amr_web_ui']),
        ('share/amr_web_ui', ['package.xml']),
        ('share/amr_web_ui/launch', glob('launch/*.py')),
        ('share/amr_web_ui/web', glob('web/*')),
    ],
    extras_require={'test': ['pytest']},
    install_requires=['setuptools'], zip_safe=True,
    maintainer='acus02', maintainer_email='acus02@example.com', license='TODO',
    entry_points={'console_scripts': ['web_ui = amr_web_ui.server:main']},
)
