from glob import glob
from pathlib import Path

SCENARIOS = Path(__file__).resolve().parents[2] / "scenarios"

from setuptools import find_packages, setup

package_name = "robot_benchmark"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/scenarios", glob(str(SCENARIOS / "*.yaml")) + glob(str(SCENARIOS / "*.md"))),
        ("share/" + package_name + "/scenarios/instances", glob(str(SCENARIOS / "instances/*.yaml"))),
    ],
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="engineering (member 3)",
    maintainer_email="engineering@todo",
    description="基准系统契约与执行入口（实现在 E14-E16）",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "benchmark_run = robot_benchmark.run:main",
            "benchmark_batch = robot_benchmark.batch:main",
            "benchmark_summarize = robot_benchmark.summarize:main",
            "benchmark_gen_scenarios = robot_benchmark.gen_scenarios:main",
        ],
    },
)
