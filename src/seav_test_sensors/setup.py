from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'seav_test_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ensure launch files are installed
        (os.path.join('share', package_name, 'launch'), glob(os.path.join(package_name,'launch', '*.py'))),
        # Ensure config files are installed
        (os.path.join('share', package_name, 'config'), glob(os.path.join(package_name,'config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seav',
    maintainer_email='seav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	"imu_serial_publisher = seav_test_sensors.imu_serial_publisher:main",
            "distance_publisher = seav_test_sensors.distance_publisher:main",
            "odom_publisher = seav_test_sensors.odom_publisher:main"
        ],
    },
)
