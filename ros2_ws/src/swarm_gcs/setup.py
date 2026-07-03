from setuptools import find_packages, setup

package_name = 'swarm_gcs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Etay Baron',
    maintainer_email='etaybaron9999@gmail.com',
    description='Ground Control Station: RViz2 visualisation, YOLO mission state machine, analysis scripts (Python)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_dashboard = swarm_gcs.mission_dashboard:main',
        ],
    },
)
