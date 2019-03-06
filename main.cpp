#include "csv_parser.h"
#include "solution.h"
#include "initial_heuristics.h"
#include "objective.h"

#include <iostream>
#include <iterator>
#include <algorithm>

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

    std::vector<vrp::Solution> solutions = {};
    for (int8_t heuristic = static_cast<int8_t>(vrp::InitialHeuristic::Savings);
        heuristic < static_cast<int8_t>(vrp::InitialHeuristic::Last);
        ++heuristic) {
        auto heuristic_solutions = vrp::create_initial_solutions(problem,
            static_cast<vrp::InitialHeuristic>(heuristic));
        solutions.insert(solutions.end(),
            std::make_move_iterator(heuristic_solutions.begin()),
            std::make_move_iterator(heuristic_solutions.end()));
    }

    if (solutions.empty()) {
        throw std::runtime_error("no solutions found");
    }

    auto best_sln = std::min_element(solutions.cbegin(), solutions.cend(),
        [&problem] (const auto& a, const auto& b) {
            return objective(problem, a) < objective(problem, b); });
    parser.print_output(*best_sln);

    return 0;
}
