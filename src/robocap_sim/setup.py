from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robocap_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all files in the urdf directory
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Include the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='31088159+shiukaheng@users.noreply.github.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world = scripts.hello_world:main'
        ],
    },
)
