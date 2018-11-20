#include "subparsers.h"
#include "utils.h"

#include <string>
#include <sstream>
#include <map>
#include <stdexcept>

namespace {
/// Constructs "expected vs actual" string
template<typename T>
std::string expected_vs_actual(const T& expected, const T& actual) {
    std::stringstream ss;
    ss << "(expected) " << expected << " vs " << actual << " (actual)";
    return ss.str();
}

template<typename T>
std::string table(T name) {
    return std::string("table ") + std::string(name);
}

/// Throws if unexpected table is given
void throw_if_unexpected_table(
    const std::string& expected,
    const std::string& table_type) {
    if (table_type != expected) {
        std::stringstream ss;
        ss << "wrong table passed: " << expected_vs_actual(
            expected, table_type);
        throw std::runtime_error(ss.str());
    }
}

/// Throws if expected >= actual
void throw_if_small_row(size_t expected, size_t actual,
    const std::string& table, uint64_t row) {
    if (expected > actual) {
        std::stringstream ss;
        ss << "unexpected " << table << " table row " << row << " length: "
           << expected_vs_actual(expected , actual);
        throw std::runtime_error(ss.str());
    }
}

/// Throws if expected != actual
void throw_if_not_equal(size_t expected, size_t actual,
    const std::string& table, uint64_t row) {
    if (expected != actual) {
        std::stringstream ss;
        ss << "unexpected " << table << " table row " << row << " length: "
           << expected_vs_actual(expected , actual);
        throw std::runtime_error(ss.str());
    }
}

/// Throws if id <= 0
void throw_if_id_not_positive(const std::string& id,
    const std::string& table, uint64_t row) {
    auto converted = std::stoll(id);
    if (converted <= 0) {
        std::stringstream ss;
        ss << "table " << table << " row " << row << "has invalid id value: "
           << converted << ". expected value > 0";
        throw std::runtime_error(ss.str());
    }
}

/// Throws if section length < expected
void throw_if_small_section(uint64_t expected, uint64_t section_length,
    const std::string& name = "") {
    if (section_length < expected) {
        std::stringstream ss;
        ss << name << " section is smaller than expected, nothing to parse: "
           << expected_vs_actual(expected, section_length);
        throw std::runtime_error(ss.str());
    }
}
}

namespace vrp {
namespace detail {
constexpr char CustomerTableParser::table_name[];
CustomerTableParser::CustomerTableParser(
    const std::vector<std::string>& raw_data,
    const std::pair<uint64_t, uint64_t>& table_section,
    size_t row_length, char delimiter) : m_delimiter(delimiter) {
    // table format:
    // "table customer" - table type
    // header
    // values:  id, demand, hard_tw_begin, hard_tw_end, soft_tw_begin, soft_tw_end,
    //          service_time, suitable_vehicles...
    throw_if_unexpected_table(this->table_name, raw_data[table_section.first]);
    throw_if_small_section(3, table_section.second - table_section.first,
        table(this->table_name));
    for (uint64_t i = table_section.first + 2; i < table_section.second; ++i) {
        auto values = split(raw_data[i], this->m_delimiter);
        throw_if_small_row(row_length, values.size(), this->table_name,
            i - (table_section.first + 2));
        throw_if_id_not_positive(values[0], this->table_name,
            i - (table_section.first + 2));
        auto vehicles = std::vector<std::string>(
            values.cbegin() + 7, values.cend());
        std::vector<uint64_t> suitable = {};
        suitable.reserve(vehicles.size());
        for (const auto& v : vehicles) {
            suitable.push_back(std::stoull(v));
        }
        this->customers.push_back({
            std::stoull(values[0]),
            std::stoull(values[1]),
            std::make_pair(std::stoull(values[2]), std::stoull(values[3])),
            std::make_pair(std::stoull(values[4]), std::stoull(values[5])),
            std::stoull(values[6]),
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
    size_t row_length, char delimiter) : m_delimiter(delimiter) {
    // table format:
    // "table vehicle" - table type
    // header
    // values: id, capacity, fixed_cost, variable_cost
    throw_if_unexpected_table(this->table_name, raw_data[table_section.first]);
    throw_if_small_section(3, table_section.second - table_section.first,
        table(this->table_name));
    for (uint64_t i = table_section.first + 2; i < table_section.second; ++i) {
        auto values = split(raw_data[i], this->m_delimiter);
        throw_if_not_equal(row_length, values.size(), this->table_name,
            i - (table_section.first + 2));
        throw_if_id_not_positive(values[0], this->table_name,
            i - (table_section.first + 2));
        this->vehicles.push_back({
            std::stoull(values[0]),
            std::stoull(values[1]),
            std::stoull(values[2]),
            std::stod(values[3]),
            std::stod(values[4])
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
    size_t row_length, char delimiter) : m_delimiter(delimiter) {
    // table format:
    // "table cost" - table type
    // values: matrix NxN where N is the number of customers
    throw_if_unexpected_table(this->table_name, raw_data[table_section.first]);
    throw_if_small_section(2,
        table_section.second - table_section.first, table(this->table_name));
    for (uint64_t i = table_section.first + 1; i < table_section.second; ++i) {
        auto values = split(raw_data[i], this->m_delimiter);
        throw_if_not_equal(row_length, values.size(), this->table_name,
            i - (table_section.first + 1));
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
    size_t row_length, char delimiter) : m_delimiter(delimiter) {
    // table format:
    // "table time" - table type
    // values: matrix NxN where N is the number of customers
    throw_if_unexpected_table(this->table_name, raw_data[table_section.first]);
    throw_if_small_section(2,
        table_section.second - table_section.first, table(this->table_name));
    for (uint64_t i = table_section.first + 1; i < table_section.second; ++i) {
        auto values = split(raw_data[i], this->m_delimiter);
        throw_if_not_equal(row_length, values.size(), this->table_name,
            i - (table_section.first + 1));
        this->times.push_back({});
        std::transform(values.cbegin(), values.cend(),
            std::back_inserter(this->times.back()),
            [](const std::string& s) { return std::stod(s); });
}
}

std::vector<std::vector<double>> TimeTableParser::get() const {
    return this->times;
}

UInt64ValueParser::UInt64ValueParser(const std::vector<std::string>& raw_data,
    const std::pair<uint64_t, uint64_t>& value_section, size_t row_length,
    char delimiter) : m_delimiter(delimiter) {
    // value format:
    // "value <name>"
    // value (uint64_t)
    throw_if_small_section(2, value_section.second - value_section.first);
    auto values = split(raw_data[value_section.first + 1], this->m_delimiter);
    throw_if_not_equal(row_length, values.size(), "",
        (value_section.first + 1));
    this->value = std::stoull(values[0]);
}

uint64_t UInt64ValueParser::get() const {
    return value;
}
}  // detail
}  // vrp
