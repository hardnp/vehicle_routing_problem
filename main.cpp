#include "constraints.h"
#include "csv_parser.h"
#include "improvement_heuristics.h"
#include "initial_heuristics.h"
#include "logging.h"
#include "objective.h"
#include "solution.h"
#include "threading.h"
#include "transportation_quantity.h"

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

void print_main_info(const vrp::Problem& prob, const vrp::Solution& sln,
                     const std::string& name) {
    LOG_INFO << name << " solution satisfies Capacity: "
             << vrp::constraints::satisfies_capacity(prob, sln) << EOL;
    LOG_INFO << name << " solution satisfies Site-Dependency: "
             << vrp::constraints::satisfies_site_dependency(prob, sln) << EOL;
    LOG_INFO << name << " solution satisfies Time Windows: "
             << vrp::constraints::satisfies_time_windows(prob, sln) << EOL;
    LOG_INFO << name << " solution's total violated time: "
             << vrp::constraints::total_violated_time(prob, sln) << EOL;
    LOG_INFO << "Objective = " << vrp::objective(prob, sln) << EOL;
    LOG_INFO << "Cost func = " << vrp::cost_function(prob, sln) << EOL;
}

void print_fmt(double objective, int violated_time,
               vrp::TransportationQuantity violated_q) {
    LOG_DEBUG << " SOLUTION: " << objective << " | " << violated_time << " | "
              << violated_q << EOL;
}

void deduplicate(const vrp::Problem& prob, std::vector<vrp::Solution>& slns) {
    std::sort(slns.begin(), slns.end(),
              [&prob](const auto& a, const auto& b) -> bool {
                  return objective(prob, a) < objective(prob, b);
              });
    auto last = std::unique(slns.begin(), slns.end());
    slns.erase(last, slns.end());
}

constexpr const size_t INITIAL_SLN_COUNT = 20;

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

    bool print_debug_info = [](char* c) {
        if (!c) {
            return false;
        }
        auto env_val = std::string(c);
        return env_val == "YES" || env_val == "Y" || env_val == "1";
    }(std::getenv("PRINT_DEBUG_INFO"));

    vrp::CsvParser parser(delimiter);
    FileHandler input(argv[1]);
    auto problem = parser.read(input.get());

    std::vector<vrp::InitialHeuristic> initial_heuristics = {
        vrp::InitialHeuristic::Savings, vrp::InitialHeuristic::Insertion,
        vrp::InitialHeuristic::ParallelInsertion,
        vrp::InitialHeuristic::ClusterFirstRouteSecond};

    // only use initial heuristics that solve split delivery problem
    if (problem.enable_splits()) {
        initial_heuristics = {vrp::InitialHeuristic::ClusterFirstRouteSecond};
    }

    std::vector<vrp::Solution> solutions = {};
    for (auto heuristic : initial_heuristics) {
        auto heuristic_solutions = vrp::create_initial_solutions(
            problem, heuristic, INITIAL_SLN_COUNT);
        solutions.insert(solutions.end(),
                         std::make_move_iterator(heuristic_solutions.begin()),
                         std::make_move_iterator(heuristic_solutions.end()));
    }

    if (solutions.empty()) {
        throw std::runtime_error("no initial solutions were created");
    }

    // delete equal initial solutions
    deduplicate(problem, solutions);

    if (print_debug_info) {
        auto best_initial_sln = *std::min_element(
            solutions.cbegin(), solutions.cend(),
            [&problem](const auto& a, const auto& b) {
                return objective(problem, a) < objective(problem, b);
            });
        print_main_info(problem, best_initial_sln, "Initial");
    }

    std::vector<vrp::Solution> improved_solutions(solutions.size());
    vrp::threading::parallel_range(improved_solutions.size(), [&](size_t first,
                                                                  size_t last) {
        for (; first != last; ++first) {
            improved_solutions[first] = std::move(create_improved_solution(
                problem, solutions[first], vrp::ImprovementHeuristic::Tabu));
        }
    });

    // delete equal improved solutions
    deduplicate(problem, improved_solutions);

    std::vector<vrp::Solution> feasible_solutions;
    feasible_solutions.reserve(improved_solutions.size());
    for (const auto& sln : improved_solutions) {
        if (!vrp::constraints::satisfies_all(problem, sln)) {
            continue;
        }
        feasible_solutions.emplace_back(sln);
    }
    // if there are no solutions that satisfy constraints, choose between all
    // found
    if (feasible_solutions.empty()) {
        feasible_solutions = improved_solutions;
    }

    auto best_sln = *std::min_element(
        feasible_solutions.cbegin(), feasible_solutions.cend(),
        [&problem](const auto& a, const auto& b) {
            return objective(problem, a) < objective(problem, b);
        });

    best_sln.update_times(problem);  // set times in case they're unset

    parser.write(std::cout, problem, best_sln);

    if (print_debug_info) {
        print_main_info(problem, best_sln, "Improved");
        print_fmt(objective(problem, best_sln),
                  vrp::constraints::total_violated_time(problem, best_sln),
                  vrp::constraints::total_violated_capacity(problem, best_sln));
    }

    return 0;
}
