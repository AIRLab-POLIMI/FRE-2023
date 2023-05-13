from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'grasslammer2_nav_py'

#packages=find_packages(exclude=['test', 'launch', 'rviz', 'test'])

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teozeta',
    maintainer_email='mtt.zinzani@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_reader = grasslammer2_nav_py.laser_reader:main',
            'pub_goal_pose = grasslammer2_nav_py.pub_navigation_goals:main',
            'converter_cmd_vel_sim = grasslammer2_nav_py.convert_cmd_vel_for_sim:main',
            'turner_from_command = grasslammer2_nav_py.turning:main',
            'pub_starting_pose = grasslammer2_nav_py.end_of_line_pose:main',
            'pub_goal_pose_def = grasslammer2_nav_py.definitive_turner:main',
            'in_row_navigation = grasslammer2_nav_py.in_row_navigation:main',
            'spray_switch = grasslammer2_nav_py.spray_switch:main',
        ],
    },
)
