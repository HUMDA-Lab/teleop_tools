from setuptools import find_packages, setup

package_name = 'can_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['dbc/EAV24_CAN2.dbc']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humdalab',
    maintainer_email='levente.puskas@humda.hu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_teleop_executable = can_teleop.can_teleop:main'
        ],
    },
)