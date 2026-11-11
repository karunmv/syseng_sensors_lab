from glob import glob
from setuptools import find_packages, setup

setup(
    name="sensor_gui",
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/sensor_gui"]),
        ("share/sensor_gui", ["package.xml", "plugin.xml"]),
        ("share/sensor_gui/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Sensors Lab serial bridge and rqt dashboard",
    entry_points={"console_scripts": ["serial_bridge = sensor_gui.bridge:main"]},
)
