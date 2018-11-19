#include "table_parsers.h"
#include "utils.h"

#include <string>
#include <map>
#include <stdexcept>

namespace {
/// Constructs "expected vs actual" string
std::string expected_vs_actual(
    const std::string& expected, const std::string& actual) {
    return std::string("(expected) ")
        + expected
        + std::string(" vs ")
        + actual
        + std::string(" (actual)");
}

/// Constructs "expected vs actual" string for integers
std::string expected_vs_actual(uint64_t expected, uint64_t actual) {
    return std::string("(expected) ")
        + std::to_string(expected)
        + std::string(" vs ")
        + std::to_string(actual)
        + std::string(" (actual)");
}

/// Throws if unexpected table is given
void throw_on_unexpected_table(
    const std::string& expected,
    const std::string& table_type_row) {
    auto table_type = vrp::detail::split(table_type_row, ',')[1];
    if (table_type != expected) {
        throw std::runtime_error(std::string(std::string("wrong table passed: ")
            + expected_vs_actual(expected, table_type)));
    }
}

/// Throws if expected != actual
void throw_not_equal(
    const std::string& table, uint64_t index, size_t expected, size_t actual) {
    if (expected != actual) throw std::runtime_error(
        std::string("unexpected ")
        + table
        + std::string(" table row ")
        + std::to_string(index)
        + std::string(" length: ")
        + expected_vs_actual(expected , actual));
}
}

namespace vrp {
namespace detail {
constexpr char CustomerTableParser::table_name[];
CustomerTableParser::CustomerTableParser(
    const std::vector<std::string>& raw_data,
    const std::pair<uint64_t, uint64_t>& table_section,
    size_t row_length) {
// table format:
// table, "customer" - table type
// header
// values: id, demand, hard_tw, soft_tw, service_time, suitable_vehicles
throw_on_unexpected_table(this->table_name, raw_data[table_section.first]);
for (uint64_t i = table_section.first + 2; i < table_section.second; ++i) {
    auto values = split(raw_data[i], ',');
    throw_not_equal(this->table_name, i - (table_section.first + 2),
        row_length, values.size());
    auto hard_tw = split(values[2], '-');
    auto soft_tw = split(values[3], '-');
    auto vehicles = split(values[5], ';');
    std::vector<uint64_t> suitable = {};
    suitable.reserve(vehicles.size());
    for (const auto& v : vehicles) {
        suitable.push_back(std::stoull(v));
    }
    this->customers.push_back({
        std::stoull(values[0]),
        std::stoull(values[1]),
        std::make_pair(std::stoull(hard_tw[0]), std::stoull(hard_tw[1])),
        std::make_pair(std::stoull(soft_tw[0]), std::stoull(soft_tw[1])),
        std::stoull(values[4]),
        suitable
    });
}
}

std::vector<Customer> CustomerTableParser::get() const {
    return this->customers;
}

constexpr char VehicleTableParser::table_name[];
VehicleTableParser::VehicleTableParser(
    const std::vector<std::string>& raw_data,
    const std::pair<uint64_t, uint64_t>& table_section,
    size_t row_length) {
// table format:
// table, "vehicle" - table type
// header
// values: id, capacity, fixed_cost, variable_cost
throw_on_unexpected_table(this->table_name, raw_data[table_section.first]);
for (uint64_t i = table_section.first + 2; i < table_section.second; ++i) {
    auto values = split(raw_data[i], ',');
    throw_not_equal(this->table_name, i - (table_section.first + 2),
        row_length, values.size());
    this->vehicles.push_back({
        std::stoull(values[0]),
        std::stoull(values[1]),
        std::stod(values[2]),
        std::stod(values[3])
    });
}
}

std::vector<Vehicle> VehicleTableParser::get() const {
    return this->vehicles;
}

constexpr char CostTableParser::table_name[];
CostTableParser::CostTableParser(
    const std::vector<std::string>& raw_data,
    const std::pair<uint64_t, uint64_t>& table_section,
    size_t row_length) {
// table format:
// table, "cost" - table type
// values: matrix NxN where N is the number of customers
throw_on_unexpected_table(this->table_name, raw_data[table_section.first]);
for (uint64_t i = table_section.first + 1; i < table_section.second; ++i) {
    auto values = split(raw_data[i], ',');
    throw_not_equal(this->table_name, i - (table_section.first + 1),
        row_length, values.size());
    this->costs.push_back({});
    std::transform(values.cbegin(), values.cend(),
        std::back_inserter(this->costs.back()),
        [](const std::string& s) { return std::stod(s); });
}
}

std::vector<std::vector<double>> CostTableParser::get() const {
    return this->costs;
}

constexpr char TimeTableParser::table_name[];
TimeTableParser::TimeTableParser(
    const std::vector<std::string>& raw_data,
    const std::pair<uint64_t, uint64_t>& table_section,
    size_t row_length) {
// table format:
// table, "time" - table type
// values: matrix NxN where N is the number of customers
throw_on_unexpected_table(this->table_name, raw_data[table_section.first]);
for (uint64_t i = table_section.first + 1; i < table_section.second; ++i) {
    auto values = split(raw_data[i], ',');
    throw_not_equal(this->table_name, i - (table_section.first + 1),
        row_length, values.size());
    this->times.push_back({});
    std::transform(values.cbegin(), values.cend(),
        std::back_inserter(this->times.back()),
        [](const std::string& s) { return std::stod(s); });
}
}

std::vector<std::vector<double>> TimeTableParser::get() const {
    return this->times;
}
}  // detail
}  // vrp
