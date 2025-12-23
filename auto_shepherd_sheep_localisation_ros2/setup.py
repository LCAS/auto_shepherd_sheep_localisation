from setuptools import setup
from glob import glob
import os

package_name = "auto_shepherd_sheep_localisation_ros2"
pkg = package_name

setup(
    name=package_name,
    version="0.0.1",
    packages=[
        package_name,
        f"{package_name}.localisation",
        f"{package_name}.detection_process",
        f"{package_name}.detection_process.modules",
    ],
    package_data={
        package_name: [
            "detection_process/models/*.pt",
            "detection_process/models/**/*.pt",
            "detection_process/modules/*.yaml",
            "web_templates/*.html",
            "web_static/*",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{pkg}"]),
        (f"share/{pkg}/config", glob(os.path.join("config", "*.rviz"))),
        (f"share/{pkg}/config", glob(os.path.join("config", "*.yaml"))),
        (f"share/{pkg}", ["package.xml"]),
    ],
    install_requires=["setuptools", "flask", "flask-socketio", "python-socketio"],
    zip_safe=False,
    maintainer="james",
    maintainer_email="primordia@live.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            f"data_loader_node.py = {pkg}.data_loader_node:main",
            f"detect_sheep.py = {pkg}.detect_sheep:main",
            f"map_visualiser_node.py = {pkg}.map_visualiser_node:main",
        ],
    },
)
