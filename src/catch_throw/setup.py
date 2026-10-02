from glob import glob

from setuptools import setup

setup(
    name="catch_throw",
    version="0.1.0",
    packages=["catch_throw"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/catch_throw"]),
                ("share/catch_throw", ["package.xml"]),
                ("share/catch_throw/config", glob("config/*.yaml") + glob("config/*.json")),
                ("share/catch_throw/launch", glob("launch/*.py"))],
    install_requires=["setuptools"],
    zip_safe=True,
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "throw_node = catch_throw.throw_node:main",
    ]},
)
