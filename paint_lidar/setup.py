from setuptools import setup

package_name = 'paint_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nik',
    maintainer_email='nikrus333@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'paint_wall = paint_lidar.paint_wall:main',
            'cliente = paint_lidar.servece_send:main',
            'cliente_copy = paint_lidar.servece_send_copy:main',
        ],
    },
)
