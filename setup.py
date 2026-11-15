from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'path_tracking_compare'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vikesz-g2662b',
    maintainer_email='Vikesz-g2662b@users.noreply.github.com',
    description='Pure Pursuit and Stanley path tracking controllers compared in simulation',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_generator = path_tracking_compare.path_generator:main',
        ],
    },
)
