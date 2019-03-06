#pragma once

#include "customer.h"
#include "vehicle.h"

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace vrp {
namespace detail {
/// Base class with common parsing checks
class BaseParser {
protected:
    std::vector<std::vector<std::string>> m_raw_values = {};
    const char m_delimiter = ';';

public:
    BaseParser(std::string name, const std::vector<std::string>& raw_data,
               const std::pair<int, int>& section, int min_section_size,
               size_t row_length, char delimiter = ';', int section_offset = 0);
};

/// Customers table parser
/*!
 *
 * Format:
 *  > "table customer" - table type
 *  > header
 *  > id, demand, hard_tw_begin, hard_tw_end, soft_tw_begin, soft_tw_end,
 *    service_time, suitable_vehicles...
 */
class CustomerTableParser : public BaseParser {
    std::vector<Customer> customers = {};

public:
    static constexpr char table_name[] = "customer";

    CustomerTableParser(const std::vector<std::string>& raw_data,
                        const std::pair<int, int>& table_section,
                        size_t row_length, char delimiter = ';');

    std::vector<Customer> get() const;
};

/// Vehicles table parser
/*!
 *
 * Format:
 *  > "table vehicle" - table type
 *  > header
 *  > id, capacity, fixed_cost, variable_cost
 */
class VehicleTableParser : public BaseParser {
    std::vector<Vehicle> vehicles = {};

public:
    static constexpr char table_name[] = "vehicle";

    VehicleTableParser(const std::vector<std::string>& raw_data,
                       const std::pair<int, int>& table_section,
                       size_t row_length, char delimiter = ';');

    std::vector<Vehicle> get() const;
};

/// Costs table parser
/*!
 *
 * Format:
 *  > "table cost" - table type
 *  > matrix NxN where N is the number of customers
 */
class CostTableParser : public BaseParser {
    std::vector<std::vector<double>> costs = {};

public:
    static constexpr char table_name[] = "cost";

    CostTableParser(const std::vector<std::string>& raw_data,
                    const std::pair<int, int>& table_section, size_t row_length,
                    char delimiter = ';');

    std::vector<std::vector<double>> get() const;
};

/// Time table parser
/*!
 *
 * Format:
 *  > "table time" - table type
 *  > matrix NxN where N is the number of customers
 */
class TimeTableParser : public BaseParser {
    std::vector<std::vector<int>> times = {};

public:
    static constexpr char table_name[] = "time";

    TimeTableParser(const std::vector<std::string>& raw_data,
                    const std::pair<int, int>& table_section, size_t row_length,
                    char delimiter = ';');

    std::vector<std::vector<int>> get() const;
};

/// 64-bit unsigned integer parser
/*!
 *
 * Format:
 *  > "value <name>" - value name
 *  > int value
 */
class IntValueParser : public BaseParser {
    int value = std::numeric_limits<int>::max();

public:
    IntValueParser(const std::vector<std::string>& raw_data,
                   const std::pair<int, int>& value_section, size_t row_length,
                   char delimiter = ';');

    int get() const;
};
}  // namespace detail
}  // namespace vrp
