from setuptools import setup

package_name = 'stairs_visualizer_pkg'

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
    description='Stairs visualizer node, publishes visualization messages',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stairs-visualizer = stairs_visualizer_pkg.stairs_visualizer:main',
        ],
    },
)
