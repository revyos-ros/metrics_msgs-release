from setuptools import setup

package_name = 'metro_benchmark_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='David V. Lu!!',
    maintainer_email='davidvlu@gmail.com',
    description='Utilities for publishing / processing metro_benchmark_msgs',
    license='BSD 3-clause',
)
