from glob import glob

from setuptools import find_packages
from setuptools import setup


PACKAGE_NAME = "amr_controller"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (
            f"share/{PACKAGE_NAME}",
            ["package.xml"],
        ),
        (
            f"share/{PACKAGE_NAME}/config",
            glob("config/*.yaml"),
        ),
        (
            f"share/{PACKAGE_NAME}/launch",
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey-c2",
    maintainer_email=(
        "rokey-c2@users.noreply.github.com"
    ),
    description="Forklift AMR ROS 2 controller",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "amr_controller = "
                "amr_controller.amr_controller:main"
            ),
        ],
    },
)
