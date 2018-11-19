#include "csv_parser.h"

/// Specify unused variable
#define UNUSED(x) (void)x;

namespace vrp {
    CsvParser::CsvParser(const std::string& file_path) : path(file_path) {
    }

    CsvParser::ParsedInputData CsvParser::load_input() const {
        return {};
    }

    void CsvParser::save_output(const Solution& sln) const {
        UNUSED(sln)
        return;
    }
}
