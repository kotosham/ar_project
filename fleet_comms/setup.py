from setuptools import find_packages, setup

package_name = 'fleet_comms'

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
    description='Shared cross-link QoS profiles + Heartbeat producer/monitor (ROADMAP Phase 1.3).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_dashboard = fleet_comms.mission_dashboard:main',
            'flat_mission_logger = fleet_comms.flat_mission_logger:main',
            'vlm_mission_logger = fleet_comms.vlm_mission_logger:main',
            'send_mission = fleet_comms.send_mission:main',
        ],
    },
)
