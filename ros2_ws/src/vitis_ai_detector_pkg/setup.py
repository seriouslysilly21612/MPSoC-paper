import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'vitis_ai_detector_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # The DPU model ships with the package that loads it, so the launch file
        # can resolve it via FindPackageShare instead of an absolute path.
        # xmodel and decode_meta.json MUST stay in the same directory: the worker
        # looks for the meta beside the xmodel.
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vitis_ai_detector_node = vitis_ai_detector_pkg.vitis_ai_detector_node:main',
        ],
    },
)
