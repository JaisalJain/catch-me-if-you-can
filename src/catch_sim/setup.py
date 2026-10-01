from glob import glob

from setuptools import setup

setup(
    name="catch_sim",
    version="0.1.0",
    packages=["catch_sim"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/catch_sim"]),
                ("share/catch_sim", ["package.xml"]),
                ("share/catch_sim/config", glob("config/*.yaml")),
                ("share/catch_sim/launch", glob("launch/*.py"))],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "scene_manager = catch_sim.scene_manager:main",
        "ground_truth_tf = catch_sim.monitors:ground_truth_main",
        "rtf_monitor = catch_sim.monitors:rtf_main",
        "gpu_check = catch_sim.monitors:gpu_main",
        "camera_tf = catch_sim.monitors:camera_tf_main",
    ]},
)
