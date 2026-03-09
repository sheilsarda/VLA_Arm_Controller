from setuptools import find_packages, setup

package_name = "vla_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/vla_params.yaml"]),
        (f"share/{package_name}/launch", ["launch/vla.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sheilsarda",
    maintainer_email="sheilsarda@gmail.com",
    description="ROS2 bridge from Isaac camera/joint topics to openpi inference and trajectory execution.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vla_controller_node = vla_controller.vla_controller_node:main",
        ],
    },
)
