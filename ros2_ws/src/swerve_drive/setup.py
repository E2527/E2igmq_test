from glob import glob

from setuptools import find_packages, setup

package_name = 'swerve_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='E2527',
    maintainer_email='e2527@matsue-ct.ac.jp',
    description='独立ステアリング(スワーブ)の足回り',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'swerve_node = swerve_drive.swerve_node:main',
            'fake_modules = swerve_drive.fake_modules:main',
            'firmware_modules = swerve_drive.firmware_modules:main',
            'keyboard_teleop = swerve_drive.keyboard_teleop:main',
            'drive_monitor = swerve_drive.drive_monitor:main',
        ],
    },
)
