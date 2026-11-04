import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'system_bringup_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # *.xml picks up fastdds_shm_profile.xml, which the launch file points
        # FASTRTPS_DEFAULT_PROFILES_FILE at. It must be installed, not just
        # present in src/, so a plain `git clone && colcon build` gets SHM
        # transport with no shell setup.
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))
            + glob(os.path.join('config', '*.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='System bringup package for KV260 pick and place pipeline',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
