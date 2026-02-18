import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'avros_webui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'static'),
            glob(os.path.join('static', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AV Lab',
    maintainer_email='avlab@cpp.edu',
    description='Phone-based joystick web UI for direct actuator control',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'webui_node = avros_webui.webui_node:main',
        ],
    },
)
