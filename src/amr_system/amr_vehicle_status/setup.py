from glob import glob
from setuptools import setup
setup(name='amr_vehicle_status', version='0.1.0', packages=['amr_vehicle_status'],
      data_files=[('share/ament_index/resource_index/packages',['resource/amr_vehicle_status']),
                  ('share/amr_vehicle_status',['package.xml','README.md']),
                  ('share/amr_vehicle_status/launch',glob('launch/*.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='acus02', maintainer_email='acus02@example.com', license='Apache-2.0',
      entry_points={'console_scripts':['vehicle_status = amr_vehicle_status.node:main']})
