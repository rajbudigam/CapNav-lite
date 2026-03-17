from setuptools import setup

package_name = 'capnav_lite_runtime'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jasraj Budigam',
    maintainer_email='jasrajharikrishna.b@indusschoolhyd.com',
    description='Runtime nodes for capability-aware assistive navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'safety_monitor_node = capnav_lite_runtime.safety_monitor_node:main',
            'shared_control_node = capnav_lite_runtime.shared_control_node:main',
            'calibration_node = capnav_lite_runtime.calibration_node:main',
        ],
    },
)
