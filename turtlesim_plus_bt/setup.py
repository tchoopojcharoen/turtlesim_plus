from setuptools import find_packages, setup

package_name = 'turtlesim_plus_bt'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/forage_bt.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pi Thanacha Choopojcharoen',
    maintainer_email='thanachachoo@gmail.com',
    description='A py_trees behavior tree that drives a turtlesim_plus turtle to forage for pizzas.',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forage_bt_node = turtlesim_plus_bt.forage_bt_node:main',
        ],
    },
)
