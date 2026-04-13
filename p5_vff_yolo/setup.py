import os
from glob import glob
from setuptools import setup

package_name = 'p5_vff_yolo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Angel Ruiz',
    maintainer_email='a.ruizf.2022@alumnos.urjc.es',
    description='P5: Follow person using VFF + YOLO with FSM (search/follow)',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'follow_person_node = p5_vff_yolo.follow_person_node:main',
        ],
    },
)
