from setuptools import find_packages, setup

package_name = 'move_in_shapes'
submodules = "move_in_shapes/helper_classes"


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools' 'shapes_interfaces'],
    zip_safe=True,
    maintainer='anna',
    maintainer_email='anna.polova@ucu.edu.ua',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_in_shapes = move_in_shapes.move_in_shapes:main',
            'shape_motion_controller = move_in_shapes.shape_motion_controller:main'
        ],
    },
)
