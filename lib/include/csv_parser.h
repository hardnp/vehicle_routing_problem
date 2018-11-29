#pragma once

#include "problem.h"
#include "solution.h"
#include "compute.h"
#include "objective.h"

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
    CsvParser(const std::string& file_path, char delimiter = ';');
    ~CsvParser();

    Problem load_input() const;

    void save_output(const std::string& out_path, const Problem& prb,
		const Solution& sln) const;
};
}  // vrp
