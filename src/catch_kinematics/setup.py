from glob import glob

from setuptools import setup

setup(
    name="catch_kinematics",
    version="2.3.0",
    packages=["catch_kinematics"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/catch_kinematics"]),
                ("share/catch_kinematics", ["package.xml"]),
                ("share/catch_kinematics/config", glob("config/*.yaml"))],
    install_requires=["setuptools"],
    zip_safe=True,
    tests_require=["pytest"],
)
