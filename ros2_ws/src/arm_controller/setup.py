from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "arm_controller"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey-c2",
    maintainer_email="rokey-c2@users.noreply.github.com",
    description="P3020 PickPlace action server bridging AMR mission requests to Isaac Sim",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pick_place_action_server = arm_controller.pick_place_action_server:main",
        ],
    },
)
