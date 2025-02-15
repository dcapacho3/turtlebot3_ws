from setuptools import find_packages, setup

package_name = 'turtlebot3_added'

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
    maintainer='sara',
    maintainer_email='sara@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'scan_filter = turtlebot3_added.scanfilter:main',
         'serial_publisher = turtlebot3_added.serial_publisher:main'
        ],
    },
)
