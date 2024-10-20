from setuptools import find_packages, setup

package_name = 'turtle_sim'

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
    maintainer='ubuntu',
    maintainer_email='yymelendez@ucsd.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = turtle_sim.my_node:main'
            'turtle_cleaner = turtle_sim.turtle_cleaner:main'
            'turtle_goal = turtle_sim.turtle_goal:main'
        ],
    },
)
