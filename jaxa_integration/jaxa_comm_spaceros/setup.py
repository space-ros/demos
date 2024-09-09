from setuptools import find_packages, setup

package_name = 'jaxa_comm_spaceros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Minahil Raza',
    maintainer_email='minahilrz@gmail.com',
    description='A package containing ROS nodes for communicating with JAXA cFS bridge',
    license='Apache-2.0 license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jaxa_simple_listener = jaxa_comm_spaceros.jaxa_simple_listener:main',
            'jaxa_rover_listener = jaxa_comm_spaceros.jaxa_rover_listener:main',
            'jaxa_canadarm_listener = jaxa_comm_spaceros.jaxa_canadarm_listener:main'
        ],
    },
)
