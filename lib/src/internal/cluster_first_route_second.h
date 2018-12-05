#pragma once

#include "problem.h"
#include "solution.h"

#include <cstdint>
#include <vector>

namespace vrp {
/// Forward declaration
enum class InitialHeuristic : int8_t;
namespace detail {
std::vector<Solution> cluster_first_route_second(const Problem& prob,
    InitialHeuristic heuristic, size_t count);
}  // detail
}  // vrp
