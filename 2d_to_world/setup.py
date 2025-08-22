from setuptools import find_packages
import os
from glob import glob
from setuptools import setup

package_name = '2d_to_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
        glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), 
        glob(os.path.join('urdf', '*.urdf'))),
        
        # Install XACRO files
        (os.path.join('share', package_name, 'urdf'), 
        glob(os.path.join('urdf', '*.xacro'))),
        
        # Install meshes (if you have any)
        (os.path.join('share', package_name, 'meshes'), 
        glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'meshes'), 
        glob(os.path.join('meshes', '*.dae'))),
        (os.path.join('share', package_name, 'meshes'), 
        glob(os.path.join('meshes', '*.obj'))),
        
        # Install world files (if you have any)
        (os.path.join('share', package_name, 'worlds'), 
        glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'worlds'), 
        glob(os.path.join('worlds', '*.sdf'))),
        
        # Install config files
        (os.path.join('share', package_name, 'config'), 
        glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), 
        glob(os.path.join('config', '*.rviz'))),
        
        # Install any other resource directories you might have
        # (os.path.join('share', package_name, 'materials'), 
        #  glob(os.path.join('materials', '*.material'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='divya',
    maintainer_email='divya.thakkar@petpooja.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add your Python executables here if any
             '2d_to_world = 2d_to_world.2d_to_world:main',
        ],
    },
)
