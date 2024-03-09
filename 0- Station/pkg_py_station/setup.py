from setuptools import find_packages, setup

package_name = 'pkg_py_station'

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
    maintainer='nadamamdouh',
    maintainer_email='nada20011002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "GUIpi = pkg_py_station.Pi:main",
            "GUIstation = pkg_py_station.Station:main"
            
        ],
    },
)
