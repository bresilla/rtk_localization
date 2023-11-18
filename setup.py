from glob import glob
from setuptools import setup
import os

package_name = 'rtk_localization'

data_files = []
#resources
data_files.append(('share/' + package_name + '/resource', glob('resource/*')))
data_files.append(('share/' + package_name, ['package.xml']))

#launch files
data_files.append((os.path.join('share', package_name), glob('launch/*.py')))
#params
data_files.append(('share/' + package_name + '/params', glob('params/*')))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bresilla',
    maintainer_email='trim.bresilla@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        # 'console_scripts': [
        #     'rtk_transform = rtk_localization.rtk_transform:main',
        #     'rtk_beardist = rtk_localization.rtk_beardist:main',
        #     'rtk_odometry = rtk_localization.rtk_odometry:main',
        #     'rtk_kalman = rtk_localization.rtk_kalman:main',
        # ],
        'console_scripts': [
            'gps_fuse = rtk_localization.gps_fuse:main',
            'gps_to_enu = rtk_localization.gps_to_enu:main',
            'odometry = rtk_localization.odometry:main',
            'transform = rtk_localization.transform:main',
        ],
        'launch.frontend.launch_extension': [
            'launch_ros = launch_ros'
        ]
    },
)
