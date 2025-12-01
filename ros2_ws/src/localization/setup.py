from setuptools import find_packages, setup

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/localization.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jcolborn',
    maintainer_email='colbornfam2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dispatch_node = localization.dispatch_node:main',
            'particle_filter = localization.particle_filter:main'
        ],
    },
)
