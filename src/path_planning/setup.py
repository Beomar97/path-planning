from setuptools import find_packages, setup

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
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
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner = path_planning.path_planner:main',
            'optimization_service = path_planning.optimization_service:main',
            'cone_publisher = path_planning.mock.cone_publisher:main',
            'planned_trajectory_subscriber = path_planning.mock.planned_trajectory_subscriber:main',
            'track_plotter = path_planning.util.track_plotter:main',
            'main_globaltraj = path_planning.algorithm.optimization.main_globaltraj:optimize',
            'testing = path_planning._testing.testing_transformer:main'
        ],
    },
)
