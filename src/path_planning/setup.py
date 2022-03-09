from setuptools import setup

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
            'path_planner = path_planning.path_planner:main',
            'input_publisher = path_planning.input_publisher:main',
        ],
    },
)
