from setuptools import find_packages, setup

package_name = 'avros_perception'

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
    maintainer='AV Lab',
    maintainer_email='avlab@cpp.edu',
    description='Semantic segmentation perception: SegFormer TensorRT inference and depth projection for Nav2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'segformer_node = avros_perception.segformer_node:main',
            'depth_projection_node = avros_perception.depth_projection_node:main',
        ],
    },
)
