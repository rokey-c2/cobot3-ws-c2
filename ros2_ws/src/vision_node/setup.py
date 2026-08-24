from setuptools import find_packages, setup

package_name = 'vision_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/vision.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hwi',
    maintainer_email='rorha66@gmail.com',
    description='YOLO 박스 검출 + depth 3D 좌표 계산 (LocateBox / ConfirmGrasp 서비스 노드)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'locate_box_node = vision_node.locate_box_node:main',
        ],
    },
)
