from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_nav'

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
    description='Nav2(自動操縦)の起動ファイルと設定',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'relocalize = robot_nav.relocalize:main',
        ],
    },
)
