#pragma once

#include "problem.h"
#include "solution.h"

#include <cstdint>
#include <vector>

namespace vrp {
/// Initial heuristics types
enum class InitialHeuristic : int8_t {
    Savings = 0,
    Insertion = 1,
    ParallelInsertion = 2,
    ClusterFirstRouteSecond = 3,
    Last
};

/// Create multiple initial solutions with specified heuristic
std::vector<Solution> create_initial_solutions(const Problem& prob,
                                               InitialHeuristic heuristic,
                                               size_t count = 1);

}  // namespace vrp
