from setuptools import find_packages, setup

package_name = 'target_person'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/launch_simulated_head.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oscar Martinez',
    maintainer_email='oscar.martinez@pal-robotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_person = target_person.target_person:main',
        ],
    },
)
