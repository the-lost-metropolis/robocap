from setuptools import find_packages, setup

package_name = 'robocap_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the urdf directory and its contents
        ('share/' + package_name + '/urdf', ['urdf/frame_ultra_low_poly.stl']),
        # If you have other files in the urdf directory, include them like this
        # ('share/' + package_name + '/urdf', ['urdf/robot.urdf.xacro', 'urdf/inertial_macros.xacro', ...]),
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
            'hello_world = robocap_sim.hello_world:main'
        ],
    },
)
