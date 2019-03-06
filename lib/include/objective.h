#pragma once

#include "problem.h"
#include "solution.h"

namespace vrp {
/// Objective function interface
double objective(const Problem& prob, const Solution& sln);
}  // namespace vrp
