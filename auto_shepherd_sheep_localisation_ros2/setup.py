from setuptools import setup
from glob import glob
import os

package_name = 'auto_shepherd_sheep_localisation_ros2'
pkg = package_name

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{pkg}']),
        (f'share/{pkg}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            f'data_loader_node.py = {pkg}.data_loader_node:main',
            f'detect_sheep.py = {pkg}.detect_sheep:main'
        ],
    },
)
