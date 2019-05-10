#!/usr/bin/env python3

import os
import argparse
import subprocess
import sys
import re
from collections import defaultdict


ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) + '/../'
SEARCH_PATTERN = re.compile(
    r'\[(?P<csv>.*.csv)\]:\s(?P<obj>.*)\s\|.*\|\s\{.*\}.*',
    re.DOTALL)


def parse_args():
    """Return basic parser for command-line arguments"""
    parser = argparse.ArgumentParser("")
    parser.add_argument('logs',
                        nargs='+',
                        help='Vehicle Routing Problem instance logs(s)')
    return parser.parse_args()


def parse(lines):
    results = []
    for line in lines:
        line = line.strip()
        m = re.search(SEARCH_PATTERN, line)
        if not m:
            continue
        results.append((m.group('csv'), float(m.group('obj'))))
    return results


def main():
    """Main entry-point"""
    args = parse_args()

    first_row = []
    results = defaultdict(list)
    for log in args.logs:
        log_name = os.path.basename(log)
        first_row.append(log_name)
        with open(log, 'r') as f:
            result_per_file = parse(f.readlines())
            for csv, obj in result_per_file:
                results[csv].append((log_name, obj))

    print(','.join([''] + first_row))
    for csv, data in sorted(results.items(), key=lambda x: x[0]):
        row = []
        for i, (log_name, obj) in enumerate(data):
            if first_row.index(log_name) != i:
                raise RuntimeError('internal error: inconsistent output')
            row.append(obj)
        print(','.join([csv] + [str(e) for e in row]))

    print('\n')

    best_per_csv = []
    for _, data in sorted(results.items(), key=lambda x: x[0]):
        if (len(set([e[1] for e in data])) == 1):
            continue
        best_per_csv.append(min(data, key=lambda x: x[1])[0])

    print('[same]:', len(results) - len(best_per_csv))
    if not best_per_csv:
        print('All logs are identical')
        return

    tries = defaultdict(int)
    for tr in best_per_csv:
        tries[tr] += 1

    for tr, count in tries.items():
        print('[{tr}]: {c}'.format(tr=tr, c=count))

    print('-'*27)
    print(' best:', max(best_per_csv, key=best_per_csv.count))


if __name__ == '__main__':
    main()
