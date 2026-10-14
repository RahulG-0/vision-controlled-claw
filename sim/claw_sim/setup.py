from glob import glob

from setuptools import find_packages, setup

package_name = 'claw_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rocki',
    maintainer_email='rockisking7@gmail.com',
    description='Gazebo simulation of the five-finger claw on a 7R arm, driven by the hand tracker',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = claw_sim.teleop_node:main',
            'demo = claw_sim.demo:main',
        ],
    },
)
