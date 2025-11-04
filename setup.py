from setuptools import setup
import os
from glob import glob

package_name = 'nav'

setup(
    name=package_name,
    version='0.5.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.sh'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jyuc',
    maintainer_email='ycheng22@gmail.com',
    description='Navigation package with STM32 ESC Controller Bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stm32_bridge_node = nav.stm32_bridge_node:main', # nav is the name of the folder for src
        ],
    },
)