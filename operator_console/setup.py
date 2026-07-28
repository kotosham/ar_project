from setuptools import find_packages, setup

package_name = 'operator_console'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Без файла-маркера в resource_index пакет не виден ament_index, и
        # `ros2 run operator_console console_node` не находит исполняемый файл.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dende',
    maintainer_email='denis.deg.08@gmail.com',
    description='Operator console: setup wizard, preflight, stack control and mission entry over stdlib HTTP.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'console_node = operator_console.console_node:main',
        ],
    },
)
