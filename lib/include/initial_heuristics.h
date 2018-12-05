#pragma once

#include "solution.h"
#include "problem.h"

#include <cstdint>
#include <vector>

namespace vrp {
/// Initial heuristics types
enum class InitialHeuristic : int8_t {
    Savings = 0,
    Insertion = 1,
    ParallelInsertion = 2,
    ClusterFirstRouteSecond = 3,
    None = 100
};

/// Create multiple initial solutions with specified heuristic
std::vector<Solution> create_initial_solutions(const Problem& prob,
    InitialHeuristic heuristic, size_t count = 1);

}  // vrp
