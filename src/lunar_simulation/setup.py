from setuptools import setup, find_packages

package_name = 'lunar_simulation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulation.launch.py']),
        ('share/' + package_name + '/worlds', ['../../worlds/lunar_crater_world_15.sdf']),
        ('share/' + package_name + '/config', ['config/simulation_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arun Saravana Lakshmi Venugopal',
    maintainer_email='arun@example.com',
    description='Lunar rover simulation package for crater-based localization',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_node = lunar_simulation.simulation_node:main',
            'crater_detector = lunar_simulation.crater_detector:main',
            'localization_node = lunar_simulation.localization_node:main',
        ],
    },
)
