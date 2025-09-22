from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('**/*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sh',
    maintainer_email='sh@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'version_node =test_pkg.version_node:main',
            'start_node =test_pkg.start_node:main',
            'serial_node =test_pkg.serial_node:main',
            'pathplan_node =test_pkg.pathplan_node:main',
            'pathplan_moveit_node =test_pkg.pathplan_moveit_node:main',
            'simulation_joint_publisher =test_pkg.simulation_joint_publisher:main',
            'qt_ros_brd_node =test_pkg.qt_ros_brd_node:main',
            'qt_node =test_pkg.qt_node:main',
            # 'debug_image_relay = test_pkg.debug_image_relay:main',
        ],
        # 'gui_scripts': [
        #     'qt_interface = test_pkg.qt_interface.start_qt_interface:main',
        # ],
    },
)
