from setuptools import setup

package_name = 'depthai_best_detection'

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
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'oakd_cpu_detector = depthai_best_detection.oakd_cpu_detector:main',
		'best_detection_node = depthai_best_detection.depthai_best_detection_node:main',
                'webcam_cpu_detector = depthai_best_detection.webcam_cpu_detector:main',
                'hazard_pid_follower = depthai_best_detection.hazard_pid_follower:main',
                'forward_distance_node = depthai_best_detection.forward_distance_node:main',
                'lidar_ld06 = depthai_best_detection.lidar_ld06:main',
                'hazard_pid_follower_timer = depthai_best_detection.hazard_pid_follower_timer:main',
        ],
    },
)
