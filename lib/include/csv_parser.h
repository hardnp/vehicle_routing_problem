#pragma once

#include "customer.h"
#include "vehicle.h"
#include "solution.h"

#include <string>
#include <vector>

namespace vrp {
/// CSV file parser
class CsvParser {
    std::string path = "";  ///< csv file path
public:
    struct ParsedInputData {
        std::vector<Vehicle> vehicles = {};  ///< vehicles list
        std::vector<Customer> customers = {};  ///< customer list
        std::vector<std::vector<double>> costs = {};  ///< cost matrix
        std::vector<std::vector<double>> times = {};  ///< time matrix
    };

    CsvParser(const std::string& file_path);

    ParsedInputData load_input() const;

    void save_output(const Solution& sln) const;
};
}  // vrp
