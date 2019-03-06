#pragma once

#include "objective.h"
#include "problem.h"
#include "solution.h"

#include <istream>
#include <limits>
#include <ostream>
#include <string>
#include <vector>

namespace vrp {
/// CSV file parser
class CsvParser {
    const char m_delimiter = ';';

public:
    CsvParser(char delimiter = ';');

    Problem read(std::istream& in) const;

    void write(std::ostream& out, const Problem& prob,
               const Solution& sln) const;
};
}  // namespace vrp
