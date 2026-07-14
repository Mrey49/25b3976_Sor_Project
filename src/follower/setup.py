from setuptools import setup
from glob import glob
import os
package_name = 'follower'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'sor_track'),
            ['models/sor_track/model.sdf', 'models/sor_track/model.config']),
        (os.path.join('share', package_name, 'models', 'sor_track', 'materials', 'textures'),
            glob('models/sor_track/materials/textures/*')),
        (os.path.join('share', package_name, 'models', 'custom_turtlebot'),
            ['models/custom_turtlebot/model.sdf', 'models/custom_turtlebot/model.config']),
        (os.path.join('share', package_name, 'models', 'custom_turtlebot', 'meshes'),
            glob('models/custom_turtlebot/meshes/*')),
        (os.path.join('share', package_name, 'models', 'track2'),
            ['models/track2/model.sdf', 'models/track2/model.config']),
        (os.path.join('share', package_name, 'models', 'track2', 'materials', 'textures'),
            glob('models/track2/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Nascarella Hishida',
    maintainer_email='gabrielnhn@ufpr.br',
    description='Have a differential drive robot follow a Robotrace track by using a camera.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tester = follower.test_topic:main',
            'follower = follower.follower_node:main',
        ],
    },
)
