from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

SCENARIOS = Path(__file__).resolve().parents[2] / "scenarios"
setup(
    name="robot_benchmark",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/robot_benchmark"]),
        ("share/robot_benchmark", ["package.xml"]),
        ("share/robot_benchmark/scenarios", glob(str(SCENARIOS / "*.yaml"))),
        ("share/robot_benchmark/scenarios/instances", glob(str(SCENARIOS / "instances/*.yaml"))),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="engineering (member 3)",
    maintainer_email="member3@example.com",
    description="Deterministic scenario generator and 100 frozen benchmark instances",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "benchmark_gen_scenarios = robot_benchmark.gen_scenarios:main",
    ]},
)
