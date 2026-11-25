from glob import glob
import os
from setuptools import find_packages, setup

package_name = "mars_yard_benchmark"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Bruno",
    maintainer_email="bruno.paschoal@tum.de",
    description="In-sim reset helpers, Nav2 benchmark runner, and analysis tools for the Mars Yard rover",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "run_benchmark = mars_yard_benchmark.run_benchmark:main",
            "analysis = mars_yard_benchmark.analysis:main",
            "robustness_analysis = mars_yard_benchmark.robustness_analysis:main",
        ],
    },
)
