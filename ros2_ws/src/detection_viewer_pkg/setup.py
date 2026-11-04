from setuptools import find_packages, setup

package_name = 'detection_viewer_pkg'

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
    maintainer='ubuntu',
    maintainer_email='kww642@mju.ac.kr',
    description='Desktop-side bbox overlay viewer for the KV260 perception pipeline',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'detection_viewer_node = detection_viewer_pkg.detection_viewer_node:main',
        ],
    },
)
