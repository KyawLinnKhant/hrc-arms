from glob import glob
from setuptools import find_packages, setup

package_name = 'hrc_physics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.xml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hrc-arms',
    maintainer_email='hrc-arms@local',
    description='MuJoCo physics sidecar for the HRC-ARMS cube.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mujoco_runner = hrc_physics.mujoco_runner:main',
        ],
    },
)
