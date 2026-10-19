from glob import glob
from setuptools import setup

package_name = 'lbr_pybullet_tutorial'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pybullet>=3.2.6,<4', 'numpy>=1.21,<2'],
    zip_safe=True,
    maintainer='Siqin',
    maintainer_email='maintainer@example.com',
    description='ROS 2 LBR trajectory control with real PyBullet feedback',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'sim_node = lbr_pybullet_tutorial.sim_node:main',
        'motion_node = lbr_pybullet_tutorial.motion_node:main',
        'demo = lbr_pybullet_tutorial.demo:main',
    ]},
)
