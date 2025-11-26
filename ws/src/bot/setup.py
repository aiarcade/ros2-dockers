from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahesh',
    maintainer_email='mahesh@fisat.ac.in',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = bot.ex1pub:main',
            'listener = bot.ex1sub:main',
            'obstacle_avoider = bot.obstacle_avoider:main',
            'fake_sensors = bot.fake_sensors:main',
            'webots_driver_node = bot.webots_driver_node:main'
        ],
    },
)
