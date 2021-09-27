from setuptools import setup

package_name = 'example_subscriber_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='peter',
    maintainer_email='peter-nebe@users.noreply.github.com',
    description='Example subscriber to stairs messages',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example-subscriber = example_subscriber_pkg.example_subscriber:main',
        ],
    },
)
