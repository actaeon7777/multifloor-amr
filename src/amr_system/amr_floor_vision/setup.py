from setuptools import find_packages, setup

setup(name='amr_floor_vision', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/amr_floor_vision']),
                  ('share/amr_floor_vision', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='AMR', maintainer_email='amr@example.com', license='Apache-2.0',
      description='Offline camera floor recognition and arrival dataset collection')
