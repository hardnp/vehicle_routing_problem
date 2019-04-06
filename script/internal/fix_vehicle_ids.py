#!/usr/bin/env python3
"""
Script to fix ids in vehicle.txt file

Run: python3 script/fix_vehicle_ids.py <path to vehicle.txt file>...

Multiple files processing is supported
"""
import argparse
import os
import sys
import math
from fnmatch import fnmatch
from pathlib import Path
from collections import defaultdict


def parse_args():
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(
        description='Test folder to CSV instance converter')
    parser.add_argument('files', nargs='+')
    return parser.parse_args()


def main():
    """Main entry-point"""
    args = parse_args()
    for i, f in enumerate(args.files):
        print('[{current}/{all}] {file}'.format(
            current=i+1, all=len(args.files), file=f))
        with open(f, 'r') as vehicle_file:
            print(vehicle_file.readline().strip())
            for i, line in enumerate(vehicle_file.readlines()):
                row = line.strip().split()
                row[0] += str(i)
                print(' '.join(row))
            pass
    return 0


if __name__ == '__main__':
    main()
