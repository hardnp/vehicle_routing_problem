#pragma once

#include "problem.h"
#include "solution.h"

#include <cstdint>

namespace vrp {
/// Optimal heuristics types
enum class OptimalHeuristic : int8_t { Tabu = 0, Last };

/// Create optimal solution with specified heuristic
Solution create_optimal_solution(const Problem& prob,
                                 const Solution& initial_sln,
                                 OptimalHeuristic heuristic);

}  // namespace vrp
