from setuptools import setup

package_name = 'lift_platform'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lift.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lift',
    maintainer_email='you@example.com',
    description='ESP32 lifting platform bridge + keyboard control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_bridge = lift_platform.serial_bridge:main',
            'keyboard_teleop = lift_platform.keyboard_teleop:main',
        ],
    },
)
