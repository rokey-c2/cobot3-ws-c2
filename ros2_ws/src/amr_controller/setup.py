from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "amr_controller"


setup(
    name=PACKAGE_NAME,
    version="0.6.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey-c2",
    maintainer_email="rokey-c2@users.noreply.github.com",
    description="IW Hub ROS 2 Jazzy Nav2 navigation and safety mux",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "odom_tf_bridge = amr_controller.odom_tf_bridge:main",
            "container_mission = amr_controller.container_mission:main",
            "static_map_publisher = amr_controller.static_map_publisher:main",
            "velocity_mux = amr_controller.velocity_mux:main",
        ],
    },
)
