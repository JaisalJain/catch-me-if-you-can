from glob import glob

from setuptools import setup

setup(
    name="catch_catcher",
    version="0.1.0",
    packages=["catch_catcher"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/catch_catcher"]),
                ("share/catch_catcher", ["package.xml"]),
                ("share/catch_catcher/config", glob("config/*.yaml")),
                ("share/catch_catcher/launch", glob("launch/*.py"))],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ["catcher = catch_catcher.catcher_node:main",
                                      "reactive_catcher = catch_catcher.reactive_node:main"]},
)
