from setuptools import find_packages, setup

package_name = 'ur_move_merlab'

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
    maintainer='bcalli',
    maintainer_email='bcalli@wpi.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ur_move_simple_interface = ur_move_merlab.ur_move_simple_interface:main',
            'test_send_trajectory = ur_move_merlab.test_send_trajectory:main'
        ],
    },
)
