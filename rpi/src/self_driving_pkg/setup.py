from setuptools import find_packages, setup

package_name = 'self_driving_pkg'

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
    maintainer='aqil',
    maintainer_email='aqilfaizal2004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'motor_node = self_driving_pkg.motor_node:main',
        'inference_node = self_driving_pkg.inference_node:main',
        'image_test_node = self_driving_pkg.image_test_node:main',
        'display_node = self_driving_pkg.display_node:main',
        'teleop_node = self_driving_pkg.teleop_node:main',
        'dataset_recorder = self_driving_pkg.dataset_recorder:main',
        'detection_node =  self_driving_pkg.detection_node:main'
    ]}
)
