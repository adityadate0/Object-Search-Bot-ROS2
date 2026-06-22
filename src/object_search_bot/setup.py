from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'object_search_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'models', 'turtlebot3_waffle'), glob(os.path.join('models', 'turtlebot3_waffle', '*'))),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='adityaaneesh.date@rwu.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        
        'perception_node = object_search_bot.perception_node:main',
        'mission_control = object_search_bot.mission_control:main',
        'base_return_manager = object_search_bot.base_return_manager:main'
        
        ],
    },
)
