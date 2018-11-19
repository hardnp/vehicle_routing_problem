#include "csv_parser.h"

#include <iostream>

/// Main entry-point to solver
int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Wrong number of input arguments" << std::endl;
        std::cerr << "Usage: vrp_solver CSV_INPUT_FILE" << std::endl;
        return 1;
    }
    auto parser = vrp::CsvParser(argv[1]);
    auto input = parser.load_input();
    return 0;
}
