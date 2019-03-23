#!/usr/bin/env python3
"""
Real life folder instance converter

Run: python3 script/folder2csv.py <path to folder>...

Multiple folders conversion is supported
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
    parser.add_argument('folders', nargs='+',
                        help='Real-life test instance folder(s)')
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


def to_str(values):
    return [str(v) for v in values]


def parse_time(value):
    hours, minutes = value.split(':')
    return 60 * int(hours) + int(minutes)


def parse_bool(value):
    value = value.lower()
    return value == "y" or value == "yes" or value == "1" or value == "true" \
        or value == "+"


def parse_vehicle_category(type1, type2):
    type1, type2 = int(type1), int(type2)
    # TODO: is the logic correct?
    return type1 if type1 != 0 else type2


def parse_categories(io_stream):
    """Parse categories.txt stream"""
    _ = io_stream.readline()  # skip header

    # format: customer_category truck_type trailer_type

    # needed: customer_category truck_type trailer_type
    table_categories = defaultdict(list)
    for line in io_stream.readlines():
        line = line.strip()
        if not line:
            continue
        values = line.split()
        # TODO: use this somehow...
        c_type = int(values[0])
        v_type = parse_vehicle_category(values[1], values[2])
        table_categories[c_type].append(v_type)
    return table_categories


def parse_customer(io_stream):
    """Parse customer.txt stream"""
    _ = io_stream.readline()  # skip header

    # format: id town-id timezone tw-begin tw-end rush-begin rush-end open close
    #         type parking reload ignore-rush split freeze-serv hot-serv noref
    #         class day single-truck access-control hard-tw can-split

    # needed: id open close tw-begin tw-end type(?) reload(?) can-split
    table_customers = {}
    for line in io_stream.readlines():
        line = line.strip()
        if not line:
            continue
        values = line.split()
        c_id = int(values[0])
        hard_tw_begin, hard_tw_end = \
            parse_time(values[7]), parse_time(values[8])
        soft_tw_begin, soft_tw_end = \
            parse_time(values[3]), parse_time(values[4])
        c_type = int(values[9])
        c_can_split = parse_bool(values[-1])
        # TODO: where's service time?
        table_customers[c_id] = (hard_tw_begin, hard_tw_end, soft_tw_begin,
                                 soft_tw_end, c_type, c_can_split)
    return table_customers


def parse_demand(io_stream):
    """Parse demand.txt stream"""
    _ = io_stream.readline()  # skip header

    # format: date object-code ref non-ref

    # needed: obect-code ref+non-ref
    table_demand = {}
    for line in io_stream.readlines():
        line = line.strip()
        if not line:
            continue
        values = line.split()
        c_id = int(values[1])
        demand = int(values[2]) + int(values[3])
        table_demand[c_id] = demand
    return table_demand


def parse_distance(io_stream):
    """Parse distance.txt stream"""
    table_distances = []
    for line in io_stream.readlines():
        line = line.strip()
        if not line:
            continue
        values = line.split()
        c_from, c_to = int(values[0]), int(values[1])
        cost, time = float(values[2]), parse_time(values[3])
        table_distances.append((c_from, c_to, cost, time))
    return table_distances


def parse_vehicle(io_stream):
    """Parse vehicle.txt stream"""
    _ = io_stream.readline()  # skip header

    # format: ID truck_category trailer_category isRef(Y/N) truck_capacity
    #         trailer_capacity parking_time join_time reload_time fixedCost
    #         balance

    # needed: ID category truck_capacity+trailer_capacity
    #         parking_time+join_time reload_time fixedCost
    table_vehicle = {}
    for line in io_stream.readlines():
        line = line.strip()
        if not line:
            continue
        values = line.split()
        v_id = int(values[0][3:])  # skip "CAR" part
        v_type1, v_type2 = values[1], values[2]
        if v_type1 != v_type2:
            raise RuntimeError("truck type != trailer type")
        v_type = parse_vehicle_category(v_type1, v_type2)
        # TODO: is this correct? may be VALUE1,VALUE2 represent volume+weight,
        # not floating value of capacity?
        capacity = float(values[4].replace(',', '.')) \
            + float(values[5].replace(',', '.'))
        work_time = sum([float(v.replace(',', '.')) for v in values[6:9]])
        fixed_cost = float(values[9])
        table_vehicle[v_id] = (v_type, capacity, work_time, fixed_cost)
    return table_vehicle


PARSERS = {
    'categories': parse_categories,
    'customer': parse_customer,
    'demand': parse_demand,
    'distance': parse_distance,
    'vehicle': parse_vehicle
}


def parse_real_life_folder(folder):
    """Parse Real Life instance folder"""
    tables = {}
    for file in os.listdir(folder):
        for name in PARSERS.keys():
            if fnmatch(file, '*{name}*.txt'.format(name=name)):
                file_path = os.path.join(folder, file)
                with open(file_path, 'r') as real_life_instance_file:
                    tables[name] = PARSERS[name](real_life_instance_file)
    return tables


def write_table_customer(io_stream, tables):
    """Write table customer into provided stream"""
    io_stream.write('table customer\n')
    io_stream.write(
        'id;volume;weight;hard_tw_begin;hard_tw_end;soft_tw_begin;')
    io_stream.write('soft_tw_end;service_time;suitable_vehicles\n')

    depot_subtrahend = min(i for i in tables['customer'].keys())
    for key, values in tables['customer'].items():
        demand = tables['demand'].get(key, 0)
        table_entry = []
        table_entry.append(key - depot_subtrahend)
        table_entry.append(demand)  # volume
        table_entry.append(demand)  # weight
        table_entry += values[:4]  # time windows
        suitable_types = tables['categories'][values[4]]
        suitable_vehicles = []
        for t in suitable_types:
            suitable_vehicles += \
                [k for k, v in tables['vehicle'].items() if v[0] == t]
        service_time = sum([tables['vehicle'][v][2]
                            for v in suitable_vehicles])
        if suitable_vehicles:
            service_time /= len(suitable_vehicles)

        if (float(service_time) > float(values[1]) - float(values[0])):
            raise ValueError("Customer table: service time > hard time window")

        table_entry.append(service_time)  # service_time
        table_entry += suitable_vehicles  # suitable vehicles
        io_stream.write(';'.join(to_str(table_entry)) + '\n')


def write_table_vehicles(io_stream, tables):
    """Write table vehicles into provided stream"""
    io_stream.write('table vehicle\n')
    io_stream.write('id;volume;weight;fixed_cost;variable_cost\n')

    for key, values in tables['vehicle'].items():
        table_entry = []
        table_entry.append(key)
        table_entry.append(values[1])  # volume
        table_entry.append(values[1])  # weight
        table_entry.append(values[-1])  # fixed cost
        table_entry.append(0)  # TODO: variable cost
        io_stream.write(';'.join(to_str(table_entry)) + '\n')


def write_table_costs(io_stream, tables):
    """Write table costs into provided stream"""
    io_stream.write('table cost\n')

    depot_subtrahend = min(i for i in tables['customer'].keys())
    def sub(x): return x - depot_subtrahend

    costs = {}
    n_customers = len(tables['customer'])
    for c_id in tables['customer'].keys():
        costs[sub(c_id)] = [None] * n_customers

    for values in tables['distance']:
        costs[sub(values[0])][sub(values[1])] = float(values[2])

    for key in sorted(costs):
        row = ['{val}'.format(val=round(e, 5)) for e in costs[key]]
        io_stream.write(';'.join(to_str(row)) + '\n')


def write_table_times(io_stream, tables):
    """Write table times into provided stream"""
    io_stream.write('table time\n')

    depot_subtrahend = min(i for i in tables['customer'].keys())
    def sub(x): return x - depot_subtrahend

    times = {}
    n_customers = len(tables['customer'])
    for c_id in tables['customer'].keys():
        times[sub(c_id)] = [None] * n_customers

    for values in tables['distance']:
        times[sub(values[0])][sub(values[1])] = int(values[3])

    for key in sorted(times):
        io_stream.write(';'.join(to_str(times[key])) + '\n')


def write_max_violated_soft_tw(io_stream):
    """Write max violated soft time-window constraints into provided stream"""
    io_stream.write('value max_violated_soft_tw\n0\n')


def main():
    """Main entry-point"""
    args = parse_args()
    test_data = os.path.abspath(
        str(Path(__file__, '../../test_data/real_life/')))
    if not os.path.isdir(test_data):
        os.makedirs(test_data)
    for i, folder in enumerate(args.folders):
        print('[{current}/{all}] {fldr}'.format(
            current=i+1, all=len(args.folders), fldr=folder))
        tables = parse_real_life_folder(folder)
        folder_basename = os.path.basename(os.path.relpath(folder))
        csv = os.path.abspath(
            str(Path(test_data, folder_basename.lower() + '.csv')))
        with open(csv, 'w+') as csv_file:
            write_table_customer(csv_file, tables)
            csv_file.write('\n')
            write_table_vehicles(csv_file, tables)
            csv_file.write('\n')
            write_table_costs(csv_file, tables)
            csv_file.write('\n')
            write_table_times(csv_file, tables)
            csv_file.write('\n')
            write_max_violated_soft_tw(csv_file)
            csv_file.write('\n')
    return 0


if __name__ == '__main__':
    main()
