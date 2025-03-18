import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ar_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pegah',
    maintainer_email='pegah@todo.todo',
    description='Package for cubic trajectory generation',
    license='Apache-2.0',
    test_require=['pytest'],
    entry_points={
        'console_scripts': [
            'points_generator = ar_test.points_generator:main',
            'cubic_traj_planner = ar_test.cubic_traj_planner:main',
            'compute_cubic_coeffs = ar_test.compute_cubic_coeffs:main',
            'plot_cubic_traj = ar_test.plot_cubic_traj:main',
        ],
    },
)

