from setuptools import find_packages, setup

package_name = 'swarm_target_detection'

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
    description='Per-drone target detection: HSV red-color-blob detector on the RGB-D camera, '
                'publishing TargetFound with an estimated 3D world position (Python)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'target_detector = swarm_target_detection.target_detector:main',
        ],
    },
)
