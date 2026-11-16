from glob import glob
from pathlib import Path
from setuptools import find_packages, setup
CORE = Path(__file__).resolve().parents[2]
# 契约路径（唯一实例表）：<repo>/docs/scenarios/instances.yaml
# setuptools 要求 data_files 用相对路径，所以构建时把它复制到包内 scenarios/ 下再安装
MANIFEST_SRC = CORE.parent / 'docs/scenarios/instances.yaml'
MANIFEST_LOCAL = CORE / 'scenarios/instances.yaml'
if MANIFEST_SRC.is_file():
    MANIFEST_LOCAL.write_bytes(MANIFEST_SRC.read_bytes())

# 说明：实例表（docs/scenarios/instances.yaml）与五份模板（scenarios/*.yaml）都**不打进 data_files**——
# setuptools 在 --symlink-install 下会把相对 data_files 解析到 build/ 目录导致安装失败。
# 生成器/校验/采集一律在仓库内运行（scenario_gen.default_manifest_path() 会按 NAV_WS 或仓库层级查找）。
install_data = [
    ('share/ament_index/resource_index/packages', ['resource/robot_benchmark']),
    ('share/robot_benchmark', ['package.xml']),
]
setup(name='robot_benchmark', version='0.2.0', packages=find_packages(exclude=['test']), data_files=install_data, install_requires=['setuptools', 'PyYAML', 'numpy'], zip_safe=True, maintainer='engineering (member 3)', maintainer_email='member3@example.com', description='Contract-v1.2 scenario table and deterministic generator', license='Apache-2.0', entry_points={'console_scripts': [
    'benchmark_gen_scenarios = robot_benchmark.gen_scenarios:main',
    'benchmark_run = robot_benchmark.runner:main',
    'benchmark_summarize = robot_benchmark.summarize:main',
]})
