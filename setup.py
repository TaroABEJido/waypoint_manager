from setuptools import setup
import os
from glob import glob

package_name = 'waypoint_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('waypoints/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TaroABE',
    maintainer_email='abet.jido.kenki@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_manager = waypoint_manager.waypoint_manager:main',
            'waypoint_saver = waypoint_manager.waypoint_saver:main',
            'waypoint_saver_for_map = waypoint_manager.waypoint_saver_for_map:main',
            'param_test = waypoint_manager.param_test:main',
            'gps_on = waypoint_manager.gps_on_amcl_off:main',
            'amcl_on = waypoint_manager.amcl_on_gps_off:main'
        ],
    },
)
