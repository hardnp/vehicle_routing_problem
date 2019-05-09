#!/usr/bin/env python3

import os
import argparse
import subprocess
import sys
import re
from collections import defaultdict


ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + '/../'
RESULT_PATTERN = re.compile(
    r'.*SOLUTION:\s*(?P<data>\d+\.\d+ \| \d+ \| \{.*\} \| \d+ \| \d).*',
    re.DOTALL)


def parse_args():
    """Return basic parser for command-line arguments"""
    parser = argparse.ArgumentParser("")
    parser.add_argument('instances',
                        nargs='+',
                        help='Vehicle Routing Problem instance file(s)')
    return parser.parse_args()


def out(lines):
    for l in lines:
        print(l)


def run_steps(steps, work_dir='.', name=None):
    """Run \n separated commands"""
    lines = []
    for step in steps.splitlines():
        step = step.strip()
        if not step:
            continue
        step_name = step.split(' ')[0]
        if name is not None:
            step_name = name
        step_str = '[{name}]'.format(name=step_name)
        print(step_str)
        sys.stdout.flush()
        result = subprocess.run(step, shell=True, cwd=work_dir,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if result.returncode != 0:
            raise RuntimeError(result.stderr)
        step_lines = result.stdout.decode('utf-8').splitlines()
        out(step_lines)
        lines += step_lines
        sys.stdout.flush()
    return lines


def parse(lines):
    results = []
    for line in lines:
        line = line.strip()
        m = re.search(RESULT_PATTERN, line)
        if not m:
            continue
        results.append("{d}".format(d=m.group('data')))
    return results


def main():
    """Main entry-point"""
    args = parse_args()

    results = {}
    for instance in args.instances:
        command = """
PRINT_DEBUG_INFO=1 time -p ./build/vrp_solver {inst}
""".format(inst=instance)
        lines = run_steps(command, ROOT_DIR, '{inst}'.format(inst=instance))
        results[os.path.basename(instance)] = parse(lines)

    print('-'*27)

    for log, data in results.items():
        if len(data) > 1:
            raise RuntimeError('Unexpected: len(data) > 1')
        print("[{l}]: {d}".format(l=log, d=data[0]))


if __name__ == '__main__':
    main()
