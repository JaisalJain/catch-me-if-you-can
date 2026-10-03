from glob import glob

from setuptools import setup

setup(
    name="catch_perception",
    version="2.3.0",
    packages=["catch_perception"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/catch_perception"]),
                ("share/catch_perception", ["package.xml"]),
                ("share/catch_perception/config", glob("config/*.yaml")),
                ("share/catch_perception/launch", glob("launch/*.py"))],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "pallet_detector = catch_perception.nodes:detector_main",
        "color_check = catch_perception.nodes:color_main",
        "eval_detection = catch_perception.eval_detection:main",
    ]},
)
