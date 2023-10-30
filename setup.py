from setuptools import find_packages, setup

package_name = 'my_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jammy',
    maintainer_email='scuartasm20@gmail.com',
    description='cmd vel publisher',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vel_publisher = my_publisher.vel_publisher:main',
            'odom_subscriber = my_publisher.odom_subscriber:main',
            'control_diff_drive = my_publisher.control_diff_drive:main'
        ],
    },
)