import os
from glob import glob
from setuptools import setup

package_name = 'grasslammer2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
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
            
            'raw_ls_rotation = grasslammer2_bringup.raw_ls_rotation:main',
            'raw_pc_rotation = grasslammer2_bringup.raw_pc_rotation:main',
        ],
    },
)
