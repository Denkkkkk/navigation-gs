from setuptools import setup
import os
from glob import glob
import subprocess

neupan_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "third_parties", "neupan")

package_name = 'neupan_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'checkpoints'), glob('checkpoints/*.pth')),
    ],
    install_requires=[
        'setuptools',
        "ecos",
        "scipy",
        "torch",
        "rich",
        "pyyaml",
        "scikit-learn",
        "gctl==1.2",
        "osqp<1.0.0",
        "numpy",
        "cvxpy",
        "diffcp==1.0.23",
        "cvxpylayers==0.1.6",
        f'neupan @ file://{neupan_path}'],
    zip_safe=True,
    maintainer='northpoleforce',
    maintainer_email='northpoleforce@qq.com',
    description='The neupan_controller package',
    license='GNU GPL v3',
    entry_points={
        'console_scripts': [
            'neupan_node = neupan_controller.neupan_node:main',
        ],
    },
)