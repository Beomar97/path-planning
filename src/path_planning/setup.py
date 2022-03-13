from setuptools import setup

package_name = 'path_planning'
submodule_model = '/model'
submodule_algorithm = '/algorithm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name, 
        package_name + submodule_model, 
        package_name + submodule_algorithm
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Forster',
    maintainer_email='forstma1@students.zhaw.ch',
    description='Path Planning package for FSZHAW autonomous system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'testing = path_planning.testing:main',
            'track_plotter = path_planning.track_plotter:main',
            'path_planner = path_planning.path_planner:main',
            'cone_publisher = path_planning.cone_publisher:main',
        ],
    },
)
