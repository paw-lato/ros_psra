from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roboracer_sim'


def package_files(directory):
    """Ładuje pliki wszystkich map"""
    entries = []
    for dirpath, _, filenames in os.walk(directory):
        if not filenames:
            continue
        entries.append((
            os.path.join('share', package_name, dirpath),
            [os.path.join(dirpath, name) for name in filenames],
        ))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        *package_files('worlds'),
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
            'pure_pursuit = roboracer_sim.pure_pursuit:main',
            'track_viz = roboracer_sim.track_viz:main',
            'line_perception = roboracer_sim.line_perception:main',
        ],
    },
)
