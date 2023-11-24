from setuptools import find_packages, setup

package_name = 'board_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # It will find Python packages automatically
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/ChessMove.srv']),  # Include service definitions
        # If you have any other data files like launch files, include them here as well
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kajc10',
    maintainer_email='katica.bozso@gmail.com',
    description='ROS 2 package to manage chess board and communicate with motion planner', 
    license='TODO: License declaration', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'board_manager_service = board_manager.board_manager_service:main',  # The executable for your node
            # Add any other console scripts for other nodes in your package if necessary
        ],
    },
)
