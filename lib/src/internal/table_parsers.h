#pragma once

#include "customer.h"
#include "vehicle.h"

#include <cstdint>
#include <string>
#include <vector>
#include <utility>

namespace vrp {
namespace detail {
class CustomerTableParser {
    std::vector<Customer> customers = {};
public:
    static constexpr char table_name[] = "customer";

    CustomerTableParser(const std::vector<std::string>& raw_data,
        const std::pair<uint64_t, uint64_t>& table_section, size_t row_length);

    ~CustomerTableParser() = default;

    std::vector<Customer> get() const;
};

class VehicleTableParser {
    std::vector<Vehicle> vehicles = {};
public:
    static constexpr char table_name[] = "vehicle";

    VehicleTableParser(const std::vector<std::string>& raw_data,
        const std::pair<uint64_t, uint64_t>& table_section, size_t row_length);

    ~VehicleTableParser() = default;

    std::vector<Vehicle> get() const;
};

class CostTableParser {
    std::vector<std::vector<double>> costs = {};
public:
    static constexpr char table_name[] = "cost";

    CostTableParser(const std::vector<std::string>& raw_data,
        const std::pair<uint64_t, uint64_t>& table_section, size_t row_length);

    ~CostTableParser() = default;

    std::vector<std::vector<double>> get() const;
};

class TimeTableParser {
    std::vector<std::vector<double>> times = {};
public:
    static constexpr char table_name[] = "time";

    TimeTableParser(const std::vector<std::string>& raw_data,
        const std::pair<uint64_t, uint64_t>& table_section, size_t row_length);

    ~TimeTableParser() = default;

    std::vector<std::vector<double>> get() const;
};
}  // detail
}  // vrp
