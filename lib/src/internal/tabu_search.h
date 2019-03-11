#pragma once

#include "problem.h"
#include "solution.h"

namespace vrp {
namespace detail {
Solution tabu_search(const Problem& prob, const Solution& initial_sln);
}  // namespace detail
}  // namespace vrp
