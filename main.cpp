#include "constraints.h"
#include "csv_parser.h"
#include "improvement_heuristics.h"
#include "initial_heuristics.h"
#include "logging.h"
#include "objective.h"
#include "solution.h"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>

namespace {
class FileHandler {
    std::ifstream m_file;

public:
    FileHandler(std::string path) : m_file(path) {
        if (!m_file.good()) {
            std::stringstream ss;
            ss << "something is wrong with the file path provided: "
               << "'" << path << "'";
            throw std::runtime_error(ss.str().c_str());
        }
    }

    std::ifstream& get() { return m_file; }

    ~FileHandler() { m_file.close(); }
};
}  // namespace

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
    vrp::CsvParser parser(delimiter);
    FileHandler input(argv[1]);
    auto problem = parser.read(input.get());

    std::vector<vrp::Solution> solutions = {};
    for (int8_t heuristic = static_cast<int8_t>(vrp::InitialHeuristic::Savings);
         heuristic < static_cast<int8_t>(vrp::InitialHeuristic::Last);
         ++heuristic) {
        auto heuristic_solutions = vrp::create_initial_solutions(
            problem, static_cast<vrp::InitialHeuristic>(heuristic));
        solutions.insert(solutions.end(),
                         std::make_move_iterator(heuristic_solutions.begin()),
                         std::make_move_iterator(heuristic_solutions.end()));
    }

    if (solutions.empty()) {
        throw std::runtime_error("no solutions found");
    }

#ifndef NDEBUG
    auto best_initial_sln = *std::min_element(
        solutions.cbegin(), solutions.cend(),
        [&problem](const auto& a, const auto& b) {
            return objective(problem, a) < objective(problem, b);
        });
    LOG_INFO << "Initial solution satisfies Site-Dependency: "
             << vrp::constraints::satisfies_site_dependency(problem,
                                                            best_initial_sln)
             << EOL;
    LOG_INFO << "Initial solution satisfies Time Windows: "
             << vrp::constraints::satisfies_time_windows(problem,
                                                         best_initial_sln)
             << EOL;
    LOG_INFO << "Objective = " << objective(problem, best_initial_sln) << EOL;
#endif

    std::vector<vrp::Solution> improved_solutions = {};
    improved_solutions.reserve(solutions.size());
    for (const auto& sln : solutions) {
        improved_solutions.emplace_back(std::move(create_improved_solution(
            problem, sln, vrp::ImprovementHeuristic::Tabu)));
    }

    auto best_sln = *std::min_element(
        improved_solutions.cbegin(), improved_solutions.cend(),
        [&problem](const auto& a, const auto& b) {
            return objective(problem, a) < objective(problem, b);
        });

    best_sln.update_times(problem);  // set times in case they're unset

    parser.write(std::cout, problem, best_sln);

#ifndef NDEBUG
    LOG_INFO << "Improved solution satisfies Site-Dependency: "
             << vrp::constraints::satisfies_site_dependency(problem, best_sln)
             << EOL;
    LOG_INFO << "Improved solution satisfies Time Windows: "
             << vrp::constraints::satisfies_time_windows(problem, best_sln)
             << EOL;
    LOG_INFO << "Objective = " << objective(problem, best_sln) << EOL;
#endif

    return 0;
}
