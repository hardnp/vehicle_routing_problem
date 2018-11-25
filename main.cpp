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
    auto problem = parser.load_input();
	auto solution = compute_solution(problem);

	parser.save_output(problem, solution);
    return 0;
}
