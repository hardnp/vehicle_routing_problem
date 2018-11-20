#pragma once

#include "customer.h"
#include "vehicle.h"
#include "solution.h"

#include <string>
#include <vector>
#include <fstream>
#include <limits>

namespace vrp {
/// CSV file parser
class CsvParser {
    const char m_delimiter = ';';
    mutable std::ifstream m_csv_file;
public:
    struct ProblemInput {
        std::vector<Vehicle> vehicles = {};  ///< vehicles list
        std::vector<Customer> customers = {};  ///< customer list
        std::vector<std::vector<double>> costs = {};  ///< cost matrix
        std::vector<std::vector<double>> times = {};  ///< time matrix
        uint64_t max_violated_soft_tw =
            std::numeric_limits<uint64_t>::max();   ///< max number of violated
                                                    /// soft time windows
    };

    CsvParser(const std::string& file_path, char delimiter = ';');
    ~CsvParser();

    ProblemInput load_input() const;

    void save_output(const Solution& sln) const;
};
}  // vrp
