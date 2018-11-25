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
BaseParser::BaseParser(std::string name,
    const std::vector<std::string>& raw_data,
    const std::pair<uint64_t, uint64_t>& section, uint64_t min_section_size,
    size_t row_length, char delimiter, uint64_t section_offset) :
    m_delimiter(delimiter) {
    /// perform basic input checks and split data into rows of string values
    if (!name.empty()) {
        throw_if_unexpected_table(name, raw_data[section.first]);
    }
    throw_if_small_section(min_section_size, section.second - section.first,
        table(name));
    auto start = section.first + section_offset;
    for (uint64_t i = start; i < section.second; ++i) {
        auto values = split(raw_data[i], this->m_delimiter);
        throw_if_small_row(row_length, values.size(), name, i - start);
        this->m_raw_values.push_back(std::move(values));
    }
}

constexpr char CustomerTableParser::table_name[];
CustomerTableParser::CustomerTableParser(
    const std::vector<std::string>& raw_data,
    const std::pair<uint64_t, uint64_t>& table_section,
    size_t row_length, char delimiter) :
    BaseParser(CustomerTableParser::table_name, raw_data, table_section, 3,
        row_length, delimiter, 2) {
    for (const auto& row : this->m_raw_values) {
        auto vehicles = std::vector<std::string>(row.cbegin() + 7, row.cend());
        std::vector<uint64_t> suitable = {};
        suitable.reserve(vehicles.size());
        for (const auto& v : vehicles) suitable.push_back(std::stoull(v));
        this->customers.push_back({
            std::stoull(row[0]),
            std::stoull(row[1]),
            std::make_pair(std::stoull(row[2]), std::stoull(row[3])),
            std::make_pair(std::stoull(row[4]), std::stoull(row[5])),
            std::stoull(row[6]),
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
    size_t row_length, char delimiter) :
    BaseParser(VehicleTableParser::table_name, raw_data, table_section, 3,
        row_length, delimiter, 2) {
    for (const auto& row : this->m_raw_values) {
        this->vehicles.push_back({
            std::stoull(row[0]),
            std::stoull(row[1]),
            std::stoull(row[2]),
            std::stod(row[3]),
            std::stod(row[4])
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
    size_t row_length, char delimiter) :
    BaseParser(CostTableParser::table_name, raw_data, table_section, 2,
        row_length, delimiter, 1) {
    for (const auto& row : this->m_raw_values) {
        this->costs.push_back({});
        std::transform(row.cbegin(), row.cend(),
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
    size_t row_length, char delimiter) :
    BaseParser(TimeTableParser::table_name, raw_data, table_section, 2,
        row_length, delimiter, 1) {
    for (const auto& row : this->m_raw_values) {
        this->times.push_back({});
        std::transform(row.cbegin(), row.cend(),
            std::back_inserter(this->times.back()),
            [](const std::string& s) { return std::stoull(s); });
    }
}

std::vector<std::vector<uint64_t>> TimeTableParser::get() const {
    return this->times;
}

UInt64ValueParser::UInt64ValueParser(const std::vector<std::string>& raw_data,
    const std::pair<uint64_t, uint64_t>& value_section, size_t row_length,
    char delimiter) :
    BaseParser("", raw_data, value_section, 2, row_length, delimiter, 1) {
    this->value = std::stoull(this->m_raw_values[0][0]);
}

uint64_t UInt64ValueParser::get() const {
    return value;
}
}  // detail
}  // vrp
