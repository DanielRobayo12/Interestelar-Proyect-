from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'interestelar_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'URDF'),glob(os.path.join('URDF', '*.xacro'))),
        (os.path.join('share',package_name, 'launch'), glob(os.path.join('launch','*launch.[pxy][yma]*'))),
        (os.path.join('share',package_name, 'rviz2'), glob(os.path.join('rviz2','*.rviz'))),
        (os.path.join('share', package_name, 'URDF', 'meshes'), glob('URDF/meshes/*.stl')),
        ('share/interestelar_pkg/config', [
        'config/bridge_config.yaml']),
        
        ('share/interestelar_pkg/worlds', [
        'worlds/world.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danirob',
    maintainer_email='daniel6968.felipe@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'n2yo_connect = interestelar_pkg.n2yo_connect:main',
            'uart_bridge = interestelar_pkg.uart_bridge:main',
        ],
    },
)
