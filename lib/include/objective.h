#pragma once

#include "problem.h"
#include "solution.h"

namespace vrp {
/// Objective function for solution
double objective(const Problem& prob, const Solution& sln);

/// Objective function for route
double objective(const Problem& prob, Solution::VehicleIndex vi,
                 const Solution::RouteType& route);
}  // namespace vrp
