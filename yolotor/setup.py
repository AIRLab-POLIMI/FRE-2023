from setuptools import setup

package_name = 'yolotor'

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
    maintainer='mirko',
    maintainer_email='mirko.usuelli@mail.polimi.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolotor_node = yolotor.yolotor_node:main',    
            'yolotor_tester = yolotor.yolotor_tester:main',
            'led_link_yolo = yolotor.yolo_led_link:main',
        ],
    },
)
