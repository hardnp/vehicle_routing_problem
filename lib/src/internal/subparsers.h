#pragma once

#include "customer.h"
#include "vehicle.h"

#include <cstdint>
#include <string>
#include <vector>
#include <utility>
#include <limits>

namespace vrp {
namespace detail {
class CustomerTableParser {
    const char m_delimiter = ';';
    std::vector<Customer> customers = {};
public:
    static constexpr char table_name[] = "customer";

    CustomerTableParser(const std::vector<std::string>& raw_data,
        const std::pair<uint64_t, uint64_t>& table_section, size_t row_length,
        char delimiter = ';');

    ~CustomerTableParser() = default;

    std::vector<Customer> get() const;
};

class VehicleTableParser {
    const char m_delimiter = ';';
    std::vector<Vehicle> vehicles = {};
public:
    static constexpr char table_name[] = "vehicle";

    VehicleTableParser(const std::vector<std::string>& raw_data,
        const std::pair<uint64_t, uint64_t>& table_section, size_t row_length,
        char delimiter = ';');

    ~VehicleTableParser() = default;

    std::vector<Vehicle> get() const;
};

class CostTableParser {
    const char m_delimiter = ';';
    std::vector<std::vector<double>> costs = {};
public:
    static constexpr char table_name[] = "cost";

    CostTableParser(const std::vector<std::string>& raw_data,
        const std::pair<uint64_t, uint64_t>& table_section, size_t row_length,
        char delimiter = ';');

    ~CostTableParser() = default;

    std::vector<std::vector<double>> get() const;
};

class TimeTableParser {
    const char m_delimiter = ';';
    std::vector<std::vector<double>> times = {};
public:
    static constexpr char table_name[] = "time";

    TimeTableParser(const std::vector<std::string>& raw_data,
        const std::pair<uint64_t, uint64_t>& table_section, size_t row_length,
        char delimiter = ';');

    ~TimeTableParser() = default;

    std::vector<std::vector<double>> get() const;
};

class UInt64ValueParser {
    const char m_delimiter = ';';
    uint64_t value = std::numeric_limits<uint64_t>::max();
public:
    UInt64ValueParser(const std::vector<std::string>& raw_data,
        const std::pair<uint64_t, uint64_t>& value_section, size_t row_length,
        char delimiter = ';');

    ~UInt64ValueParser() = default;

    uint64_t get() const;
};
}  // detail
}  // vrp
