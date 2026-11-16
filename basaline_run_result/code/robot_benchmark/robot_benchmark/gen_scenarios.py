"""Generate/check one frozen scenario table without ROS or Isaac imports."""
from __future__ import annotations
import argparse
from pathlib import Path
import yaml
from .scenario_gen import SCENARIO_KEYS, default_manifest_path, default_templates_path, generate_document, validate_document

def load_templates(root):
    return {key: yaml.safe_load((Path(root) / (key + '.yaml')).read_text(encoding='utf-8')) for key in SCENARIO_KEYS}

def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, default=default_manifest_path(), help='契约统一表的完整文件路径')
    parser.add_argument('--templates', type=Path, default=default_templates_path())
    parser.add_argument('--check', action='store_true', help='只校验，不写文件')
    args = parser.parse_args(argv)
    expected = generate_document(load_templates(args.templates))
    if args.check:
        if not args.out.is_file():
            print('缺少实例表:', args.out)
            return 2
        actual = yaml.safe_load(args.out.read_text(encoding='utf-8'))
        problems = validate_document(actual)
        if actual != expected:
            problems.append('冻结表与当前版本生成器/模板不一致')
        if problems:
            print('\n'.join(problems))
            return 2
        print('PASS: 100 个实例，契约字段、种子、A* 可达性和确定性全部通过')
        return 0
    problems = validate_document(expected)
    if problems:
        print('\n'.join(problems))
        return 2
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(yaml.safe_dump(expected, allow_unicode=True, sort_keys=False, width=100), encoding='utf-8', newline='\n')
    print('已生成 100 个实例:', args.out)
    return 0
if __name__ == '__main__':
    raise SystemExit(main())
