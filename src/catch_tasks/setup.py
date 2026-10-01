from glob import glob

from setuptools import setup

setup(
    name="catch_tasks",
    version="0.1.0",
    packages=["catch_tasks"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/catch_tasks"]),
                ("share/catch_tasks", ["package.xml"]),
                ("share/catch_tasks/config", glob("config/*.yaml")),
                ("share/catch_tasks/launch", glob("launch/*.py"))],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "jog_test = catch_tasks.jog_test:main",
        "grasp_test = catch_tasks.grasp_test:main",
        "pickplace_test = catch_tasks.pickplace_test:main",
    ]},
)
