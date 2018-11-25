#pragma once

#include "problem.h"
#include "solution.h"

class Solution;
class Problem;

namespace vrp {
/// Objective function interface
double objective(const Problem& prob, const Solution& sln);
}  // vrp
