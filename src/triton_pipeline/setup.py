from setuptools import setup
from glob import glob
import os

package_name = 'triton_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('test/config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='logan',
    maintainer_email='logan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipeline_manager = triton_pipeline.pipeline_manager:main',
            'pipeline_sequence_manager = triton_pipeline.pipeline_sequence_manager:main'
        ],
    },
)
