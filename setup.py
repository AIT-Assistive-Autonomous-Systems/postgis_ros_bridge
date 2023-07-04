from setuptools import setup

package_name = 'postgis_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'sqlalchemy>=2.0.0',
        'psycopg2',
        'pytest-postgresql',
    ],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='marco.wallner@ait.ac.at',
    description='A library for publishing spatial PostGIS data to ROS.',
    license='Apache Software License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'postgis_ros_bridge_publisher_node = '
                'postgis_ros_bridge.postgis_ros_bridge_publisher_node:main'
        ],
    },
    classifiers=[
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3 :: Only',
        'Programming Language :: Python :: 3.10',
        'Topic :: Database :: Front-Ends',
        'Topic :: Scientific/Engineering :: GIS',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    python_requires='>=3.10',
)
