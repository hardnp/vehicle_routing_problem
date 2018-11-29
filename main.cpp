#include "csv_parser.h"

#include <iostream>

/// Main entry-point to solver
int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Wrong number of input arguments" << std::endl;
        std::cerr << "Usage: vrp_solver CSV_INPUT_FILE CSV_OUTPUT_FILE [DELIMITER]"
                  << std::endl;
        return 1;
    }
    char delimiter = ';';
    if (argc > 3) {
        delimiter = argv[3][0];
    }
    vrp::CsvParser parser(argv[1], delimiter);
    auto problem = parser.load_input();
	auto solution = construct_initial_solution(problem, "savings"); //just an example

	parser.save_output(argv[2], problem, solution);
    return 0;
}
