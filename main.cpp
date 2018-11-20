#include "csv_parser.h"

#include <iostream>

/// Main entry-point to solver
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Wrong number of input arguments" << std::endl;
        std::cerr << "Usage: vrp_solver CSV_INPUT_FILE [DELIMITER]"
                  << std::endl;
        return 1;
    }
    char delimiter = ';';
    if (argc > 2) {
        delimiter = argv[2][0];
    }
    vrp::CsvParser parser(argv[1], delimiter);
    auto input = parser.load_input();
    return 0;
}
