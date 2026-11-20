from glob import glob

from setuptools import find_packages, setup

package_name = 'field_map'

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
        ('share/' + package_name + '/fields', glob('fields/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='E2527',
    maintainer_email='e2527@matsue-ct.ac.jp',
    description='フィールドの形を書いたファイルから、地図(/map)を作る',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'field_map_node = field_map.field_map_node:main',
            'make_map = field_map.make_map:main',
            'field_editor = field_map.field_editor:main',
        ],
    },
)
