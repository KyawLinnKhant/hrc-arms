from setuptools import find_packages, setup
from glob import glob

package_name = 'hrc_handoff_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz',   glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='klkhant707@gmail.com',
    description='Plan-only scripted handoff demo for HRC-ARMS.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'handoff_demo = hrc_handoff_demo.handoff_demo:main',
        ],
    },
)
