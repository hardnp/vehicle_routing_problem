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


def split_by_types(vnumber):
    """
    Split vehicle number to 3 numbers representing small, medium, big vehicles
    """
    nsmall = round(0.45 * vnumber, 0)
    nmedium = round(0.3 * vnumber, 0)
    nbig = vnumber - nsmall - nmedium
    return int(nsmall), int(nmedium), int(nbig)


def get_vehicles_from_demand(category, nsmall, nmedium, nbig):
    """Get vehicle ides for each category of customer"""
    fs, ls = 0, nsmall
    fm, lm = nsmall, nsmall+nmedium
    fb, lb = nsmall+nmedium, nsmall+nmedium+nbig
    # 0 is the smallest customer, 3 is the biggest
    vehicles_per_category = {
        0: list(range(fs, ls)),
        1: list(range(fs, ls)) + list(range(fm, lm)),
        2: list(range(fs, ls)) + list(range(fm, lm)) + list(range(fb, lb)),
        3: list(range(fm, lm)) + list(range(fb, lb))
    }
    return [str(e) for e in vehicles_per_category[category]]


def category_from_average_demand(average_demand, demand):
    demand = int(demand)
    average_demand = float(average_demand)
    if demand < average_demand:
        return 0
    elif demand < average_demand * 3 / 2:
        return 1
    elif demand < average_demand * 2:
        return 2
    else:
        return 3


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


def write_table_customer(io_stream, customers, vnumber):
    """Write table customer into provided stream"""
    io_stream.write('table customer\n')
    io_stream.write(
        'id;volume;weight;hard_tw_begin;hard_tw_end;soft_tw_begin;')
    io_stream.write('soft_tw_end;service_time;suitable_vehicles\n')
    avg_demand = sum([int(e[3]) for e in customers]) / len(customers)
    nsmall, nmedium, nbig = split_by_types(vnumber)
    for customer in customers:
        category = category_from_average_demand(avg_demand, customer[3])
        suitable = get_vehicles_from_demand(category, nsmall, nmedium, nbig)
        if int(customer[0]) == 0:
            suitable = []
        hard_tw_end = str(int(customer[6]) + int(customer[5]))
        row = customer[:1] + customer[3:4] + customer[3:5] + [hard_tw_end] +\
            customer[4:5] + [hard_tw_end] + \
            customer[6:7] + suitable
        check_customer_table_correctness(row)
        io_stream.write(';'.join(row) + '\n')


def write_table_vehicles(io_stream, vnumber, vcapacity, customers):
    """Write table vehicles into provided stream"""
    io_stream.write('table vehicle\n')
    io_stream.write('id;volume;weight;fixed_cost;variable_cost\n')
    distances = construct_distance_matrix(customers, element_type=float)
    avg_distance_cost = sum([sum(row) for row in distances]) / \
        math.pow(len(distances), 2)
    base_fixed_cost = round(avg_distance_cost * 10, 2)
    base_variable_cost = 1.0
    nsmall, nmedium, nbig = split_by_types(vnumber)
    fs, ls = 0, nsmall
    fm, lm = nsmall, nsmall+nmedium
    fb, lb = nsmall+nmedium, nsmall+nmedium+nbig
    sc, mc, bc = float(0.5), float(1.0), float(1.5)
    for i in range(fs, ls):
        row = [i, vcapacity * sc, vcapacity * sc,
               base_fixed_cost * sc, base_variable_cost * sc]
        row = [str(e) for e in row]
        io_stream.write(';'.join(row) + '\n')
    for i in range(fm, lm):
        row = [i, vcapacity * mc, vcapacity * mc,
               base_fixed_cost * mc, base_variable_cost * mc]
        row = [str(e) for e in row]
        io_stream.write(';'.join(row) + '\n')
    for i in range(fb, lb):
        row = [i, vcapacity * bc, vcapacity * bc,
               base_fixed_cost * bc, base_variable_cost * bc]
        row = [str(e) for e in row]
        io_stream.write(';'.join(row) + '\n')


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


def write_max_splits(io_stream):
    """Write max number of splits per customer into provided stream"""
    io_stream.write('value max_splits\n2\n')


def main():
    """Main entry-point"""
    args = parse_args()
    test_data = os.path.abspath(
        str(Path(__file__, '../../test_data/solomon_sdvrptwsd/')))
    if not os.path.isdir(test_data):
        os.makedirs(test_data)
    for i, instance in enumerate(args.instances):
        print('[{current}/{all}] {instance}'.format(
            current=i+1, all=len(args.instances), instance=instance))
        vnumber, vcapacity, customers = 0, 0, []
        with open(instance, 'r') as instance_file:
            # vehicles number, vehicles capacity, customers data
            vnumber, vcapacity, customers = parse_solomon_instance(
                instance_file)
        instance_basename, _ = os.path.splitext(os.path.basename(instance))
        instance_basename = '{prefix}{name}'.format(
            prefix='m', name=instance_basename)
        csv = os.path.abspath(str(Path(
            test_data, instance_basename.lower() + '.csv')))
        with open(csv, 'w+') as csv_file:
            write_table_customer(csv_file, customers, vnumber)
            csv_file.write('\n')
            write_table_vehicles(csv_file, vnumber, vcapacity, customers)
            csv_file.write('\n')
            write_table_costs(csv_file, customers)
            csv_file.write('\n')
            write_table_times(csv_file, customers)
            csv_file.write('\n')
            write_max_violated_soft_tw(csv_file)
            csv_file.write('\n')
            write_max_splits(csv_file)
            csv_file.write('\n')
    return 0


if __name__ == '__main__':
    main()
