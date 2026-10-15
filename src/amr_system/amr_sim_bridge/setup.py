from setuptools import setup
setup(name='amr_sim_bridge', version='0.1.0', packages=['amr_sim_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/amr_sim_bridge']),
                  ('share/amr_sim_bridge', ['package.xml'])],
      install_requires=['setuptools'], extras_require={'test': ['pytest']},
      description='Isaac Sim sensor and actuator bridge', license='Apache-2.0',
      maintainer='AMR Workspace', maintainer_email='amr-workspace@example.com',
      entry_points={'console_scripts': ['elevator_sim = amr_sim_bridge.elevator_node:main',
                                      'elevator_sim_relay = amr_sim_bridge.elevator_node:relay_main']})
