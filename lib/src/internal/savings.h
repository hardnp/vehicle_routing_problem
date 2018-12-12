#pragma once

#include "problem.h"
#include "solution.h"

#include <cstdint>
#include <vector>

namespace vrp {
/// Forward declaration
enum class InitialHeuristic : int8_t;
	namespace detail {
std::vector<Solution> savings(const Problem& prob, size_t count);
}  // detail
}  // vrp
