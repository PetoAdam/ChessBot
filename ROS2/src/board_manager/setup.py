from setuptools import find_packages, setup
import os

package_name = 'board_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/srv', ['srv/ChessMove.srv']),  # Include service definitions
        # If you have any other data files like launch files, include them here as well
    ],
    install_requires=['setuptools', 'flask'],  # Include Flask as a dependency
    zip_safe=True,
    maintainer='kajc10',
    maintainer_email='katica.bozso@gmail.com',
    description='ROS 2 package to manage chess board and communicate with motion planner', 
    license='TODO: License declaration', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'board_manager_service = board_manager.board_manager_service:main',
            'dummy_motion_planner = board_manager.dummy_motion_planner:main'
            # Other console scripts if necessary
        ],
    },
)
