from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roboracer_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (
            os.path.join('share', package_name, 'worlds', 'linefollower_track'),
            [
                path for path in glob('worlds/linefollower_track/*')
                if os.path.isfile(path)
            ],
        ),
        (
            os.path.join('share', package_name, 'worlds', 'linefollower_track', 'meshes'),
            [
                path for path in glob('worlds/linefollower_track/meshes/*')
                if os.path.isfile(path)
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pawel Git',
    maintainer_email='pawellato03@gmail.com',
    description='Ostra jazda',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'control_pid = roboracer_sim.control_pid:main',
          'roboracer_pid = roboracer_sim.roboracer_pid:main',
          'control_stanley = roboracer_sim.control_stanley:main',
          'evaluate_run = roboracer_sim.evaluate_run:main',
        ],
    },
)
