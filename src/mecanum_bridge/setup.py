from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mecanum_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share',package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share',package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='robotica@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_bridgeEKF = mecanum_bridge.serial_bridgeEKF:main',
            'joy_mecanum = mecanum_bridge.joy_mecanum:main',
            'teleop_mecanum = mecanum_bridge.teleop_mecanum:main',
        ],
    },
)
