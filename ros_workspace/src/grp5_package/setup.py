from setuptools import setup

package_name = 'grp5_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ee3070group5',
    maintainer_email='ctcchan8-c@my.cityu.edu.hk',
    description='RM Motor control for City University 2022 EE3070 Group 5 project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grp5_node = grp5_package.grp5_node:main'
        ],
    },
)
