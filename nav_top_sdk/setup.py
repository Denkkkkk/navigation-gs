from setuptools import find_packages, setup
import glob

package_name = 'nav_top_sdk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'ruamel.yaml'],
    zip_safe=True,
    maintainer='northpoleforce',
    maintainer_email='northpoleforce@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sdk_node = nav_top_sdk.sdk_node:main',
        ],
    },
)
