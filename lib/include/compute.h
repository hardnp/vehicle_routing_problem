#pragma once

#include "solution.h"
#include "problem.h"

namespace vrp {
    Solution construct_initial_solution(Problem prb, std::string heuristic);
    Solution construct_optimal_solution(Problem prb, Solution baseline, std::string heuristic);
}