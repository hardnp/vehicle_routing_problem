#pragma once

#include "objective.h"
#include "problem.h"
#include "solution.h"

#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace vrp {
/// CSV file parser
class CsvParser {
    const char m_delimiter = ';';
    mutable std::ifstream m_csv_file;

public:
    CsvParser(const std::string& file_path, char delimiter = ';');
    ~CsvParser();

    Problem load_input() const;

    void save_output(const std::string& out_path, const Problem& prb,
                     const Solution& sln) const;

    void print_output(const Solution& sln) const;
};
}  // namespace vrp
