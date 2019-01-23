#pragma once

#include "problem.h"
#include "solution.h"

#include <cstdint>
#include <vector>

namespace vrp {
namespace detail {
std::vector<Solution> cluster_first_route_second(const Problem& prob,
    size_t count);
}  // detail
}  // vrp
