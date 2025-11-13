from setuptools import find_packages, setup

package_name = 'tello_project'

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
    maintainer='frank',
    maintainer_email='frank@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'tello_driver = tello_project.tello_driver:main',
	        'video_viewer = tello_project.video_viewer:main',
        	'telemetry_monitor = tello_project.telemetry_monitor:main',
	        'battery_failsafe = tello_project.battery_failsafe:main',
        	'mission_planner = tello_project.mission_planner:main',
	        'object_detector = tello_project.object_detector:main',
        ],
    },
)
