from setuptools import find_packages, setup

package_name = 'tcp_package'

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
    maintainer='yusuf',
    maintainer_email='105513549+Yusuftaya1@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tcp_try  = tcp_package.tcp_deneme:main',
            'tcp_try2 = tcp_package.tcp_deneme2:main',
            'Map = tcp_package.map:main',
            'Odom= tcp_package.odom:main'

        ],
    },
)
