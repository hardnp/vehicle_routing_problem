#!/usr/bin/env python3

import os
import argparse
import subprocess
import sys
import re
from collections import defaultdict


RESULT_PATTERN = re.compile(r'.*SOLUTION:\s*(?P<data>\d+\.\d+\s\|\s\d+).*',
                            re.DOTALL)


def parse_args():
    """Return basic parser for command-line arguments"""
    parser = argparse.ArgumentParser("")
    parser.add_argument('logs',
                        nargs='+',
                        help='Vehicle Routing Problem log file(s)')
    return parser.parse_args()


def main():
    """Main entry-point"""
    args = parse_args()

    results = defaultdict(list)
    for logfile in args.logs:
        for line in open(logfile, 'r').read().splitlines():
            line = line.strip()
            m = re.search(RESULT_PATTERN, line)
            if not m:
                continue
            results[os.path.basename(logfile)].append(
                "data: {d}".format(d=m.group('data')))

    for log, result in results.items():
        print("[{l}]:".format(l=log), result)


if __name__ == '__main__':
    main()
