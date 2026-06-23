from setuptools import find_packages, setup

package_name = 'search_coordinator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dende',
    maintainer_email='denis.deg.08@gmail.com',
    description='Pi-side executive (FSM/BT) for the robust architecture (Phase 1.6 scaffold).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coordinator_node = search_coordinator.coordinator_node:main',
        ],
    },
)
