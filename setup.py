from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'BallFollower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='theolangel',
    maintainer_email='theolangel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'BallFollower = BallFollower.BallFollower:main',
            'Ball_Publisher = BallFollower.Ball_Publisher:main'
        ],
    },
)
