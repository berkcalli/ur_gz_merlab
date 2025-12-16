from setuptools import find_packages, setup

package_name = 'ur_move_custom'

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
    maintainer_email='bcalli@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ur_move_berk = ur_move_custom.ur_move_berk:main',
            'example_move = ur_move_custom.example_move:main',
            'send_pose_test = ur_move_custom.send_pose_test:main'
        ],
    },
)
