#!/usr/bin/env python3
"""
Solomon instance converter

Run: python3 script/solomon.py <path to instance>...

Multiple instance conversion is supported
"""
import argparse
import os
import sys
import math
from pathlib import Path


def parse_args():
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(
        description='Solomon to CSV instance converter')
    parser.add_argument('instances', nargs='+',
                        help='Solomon instance file(s)')
    return parser.parse_args()


def skip_lines(input_file, keyword):
    """
    Skip lines in input file until keyword is met
    Next line after keyword line is read
    """
    dummy = ''
    while True:
        dummy = input_file.readline().strip()
        if dummy == keyword:
            dummy = input_file.readline()
            break
    return input_file


def parse_solomon_instance(io_stream):
    """Parse Solomon instance file"""
    _ = io_stream.readline().strip()
    io_stream = skip_lines(io_stream, 'VEHICLE')
    number, capacity = io_stream.readline().strip().split()
    io_stream = skip_lines(io_stream, 'CUSTOMER')
    customer_data = []
    for customer_str in io_stream.readlines():
        customer_str = customer_str.strip()
        if not customer_str:
            continue
        customer_data.append(customer_str.split())
    return int(number), int(capacity), customer_data


def check_customer_table_correctness(row):
    int_row = [int(e) for e in row]
    if (int_row[7] > int_row[4] - int_row[3]):
        raise ValueError("Customer table: service time > hard time window")


def write_table_customer(io_stream, customers):
    """Write table customer into provided stream"""
    io_stream.write('table customer\n')
    io_stream.write(
        'id;volume;weight;hard_tw_begin;hard_tw_end;soft_tw_begin;')
    io_stream.write('soft_tw_end;service_time;suitable_vehicles\n')
    for customer in customers:
        row = customer[:1] + customer[3:4] + customer[3:6] + customer[4:6] \
            + customer[6:7]
        check_customer_table_correctness(row)
        io_stream.write(';'.join(row) + '\n')


def write_table_vehicles(io_stream, vnumber, vcapacity):
    """Write table vehicles into provided stream"""
    io_stream.write('table vehicle\n')
    io_stream.write('id;volume;weight;fixed_cost;variable_cost\n')
    for i in range(0, vnumber):
        row = [i, vcapacity, vcapacity, 1.0, 1.0]
        row = [str(e) for e in row]
        io_stream.write(';'.join(row) + '\n')


def construct_distance_matrix(customers, element_type=float):
    """Construct distance matrix"""
    def distance(a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    elements = []
    for i, first in enumerate(customers):
        elements.append([0]*len(customers))
        for j, second in enumerate(customers):
            point1, point2 = first[1:3], second[1:3]
            point1 = [float(e) for e in point1]
            point2 = [float(e) for e in point2]
            elements[i][j] = element_type(distance(point1, point2))
    return elements


def write_table_costs(io_stream, customers):
    """Write table costs into provided stream"""
    io_stream.write('table cost\n')
    costs = construct_distance_matrix(customers, element_type=float)
    for row in costs:
        row = ['{val}'.format(val=round(e, 5)) for e in row]
        io_stream.write(';'.join(row) + '\n')


def write_table_times(io_stream, customers):
    """Write table times into provided stream"""
    io_stream.write('table time\n')
    times = construct_distance_matrix(customers, element_type=int)
    for row in times:
        row = [str(e) for e in row]
        io_stream.write(';'.join(row) + '\n')


def write_max_violated_soft_tw(io_stream):
    """Write max violated soft time-window constraints into provided stream"""
    io_stream.write('value max_violated_soft_tw\n0\n')


def main():
    """Main entry-point"""
    args = parse_args()
    test_data = os.path.abspath(
        str(Path(__file__, '../../test_data/solomon/')))
    if not os.path.isdir(test_data):
        os.makedirs(test_data)
    for i, instance in enumerate(args.instances):
        print('[{current}/{all}] {instance}'.format(
            current=i+1, all=len(args.instances), instance=instance))
        # print('Reading {instance} ...'.format(instance=instance))
        vnumber, vcapacity, customers = 0, 0, []
        with open(instance, 'r') as instance_file:
            # vehicles number, vehicles capacity, customers data
            vnumber, vcapacity, customers = parse_solomon_instance(
                instance_file)
        instance_basename, _ = os.path.splitext(os.path.basename(instance))
        csv = os.path.abspath(str(Path(
            test_data, instance_basename.lower() + '.csv')))
        # print('Converting to {csv} ...'.format(csv=csv))
        with open(csv, 'w+') as csv_file:
            write_table_customer(csv_file, customers)
            csv_file.write('\n')
            write_table_vehicles(csv_file, vnumber, vcapacity)
            csv_file.write('\n')
            write_table_costs(csv_file, customers)
            csv_file.write('\n')
            write_table_times(csv_file, customers)
            csv_file.write('\n')
            write_max_violated_soft_tw(csv_file)
            csv_file.write('\n')
    return 0


if __name__ == '__main__':
    main()
