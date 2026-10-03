from glob import glob

from setuptools import setup

setup(
    name="catch_tasks",
    version="2.3.0",
    packages=["catch_tasks"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/catch_tasks"]),
                ("share/catch_tasks", ["package.xml"]),
                ("share/catch_tasks/config", glob("config/*.yaml")),
                ("share/catch_tasks/launch", glob("launch/*.py"))],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "jog_test = catch_tasks.jog_test:main",
        "grasp_test = catch_tasks.grasp_test:main",
        "pickplace_test = catch_tasks.pickplace_test:main",
        "throw_eval = catch_tasks.throw_eval:main",
        "catch_eval = catch_tasks.catch_eval:main",
        "run_demo = catch_tasks.state_machine:main",
        "bag_to_npz = catch_tasks.recorder:bag_to_npz",
        "tracker_eval = catch_tasks.tracker_eval:main",
        "reactive_eval = catch_tasks.reactive_eval:main",
    ]},
)
