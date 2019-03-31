#!/usr/bin/env python3

import os
import argparse
import subprocess
import sys


ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + '/../'


def parse_args():
    """Return basic parser for command-line arguments"""
    parser = argparse.ArgumentParser("")
    parser.add_argument('instances',
                        nargs='+',
                        help='Vehicle Routing Problem instance file(s)')
    return parser.parse_args()


def run_steps(steps, work_dir='.', name=None):
    """Run \n separated commands"""
    for step in steps.splitlines():
        step = step.strip()
        if not step:
            continue
        step_name = step.split(' ')[0]
        if name is not None:
            step_name = name
        sys.stdout.write('[{name}]\n'.format(name=step_name))
        subprocess.check_call(step, shell=True, cwd=work_dir)
    sys.stdout.flush()


def main():
    """Main entry-point"""
    args = parse_args()

    for instance in args.instances:
        command = """
PRINT_DEBUG_INFO=1 time -p ./build/vrp_solver {inst}
""".format(inst=instance)
        run_steps(command, ROOT_DIR, '{inst}'.format(inst=instance))


if __name__ == '__main__':
    main()
