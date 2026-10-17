from glob import glob

from setuptools import find_packages, setup

package_name = 'stm_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='e2527@matsue-ct.ac.jp',
    description='シリアルバス上の STM32 と ROS 2 トピックを中継するブリッジ',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bridge = stm_bridge.bridge_node:main',
            'fake_stm = stm_bridge.fake_stm:main',
        ],
    },
)
