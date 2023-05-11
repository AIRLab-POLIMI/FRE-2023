from setuptools import setup

package_name = 'control_int'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bido',
    maintainer_email='abdelrahmantarek.farag@mail.polimi.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
        	'pid = control_int.pid:main',
        	'pid2 = control_int.pid2:main',
        	'pid3 = control_int.pid3:main',
        	'aut_nav = control_int.aut_nav:main',
        	'aut_nav_param = control_int.aut_nav_param:main',
        ],
    },
)
