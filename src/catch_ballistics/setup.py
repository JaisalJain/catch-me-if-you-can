from setuptools import setup

setup(
    name="catch_ballistics",
    version="0.1.0",
    packages=["catch_ballistics"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/catch_ballistics"]),
                ("share/catch_ballistics", ["package.xml"])],
    install_requires=["setuptools"],
    zip_safe=True,
    tests_require=["pytest"],
)
