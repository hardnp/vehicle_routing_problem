#pragma once

#include "problem.h"
#include "solution.h"

#include <cstdint>

namespace vrp {
/// Optimal heuristics types
enum class ImprovementHeuristic : int8_t { Tabu = 0, Last };

/// Create optimal solution with specified heuristic
Solution create_improved_solution(const Problem& prob,
                                  const Solution& initial_sln,
                                  ImprovementHeuristic heuristic);

}  // namespace vrp
