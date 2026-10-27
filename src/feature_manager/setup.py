from glob import glob

from setuptools import find_packages, setup

package_name = 'feature_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yml')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools', 'PyYAML', 'docker', 'fastapi', 'uvicorn'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='max.stahlke@gmail.com',
    description='Monitors the robot feature containers and starts/stops them with options '
                'from a web UI and ROS topics.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'feature_manager = feature_manager.main:main',
        ],
    },
)
