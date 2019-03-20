#pragma once

#include "problem.h"
#include "solution.h"

namespace vrp {
namespace constraints {
int total_violated_time(const Problem& prob, const Solution& sln);

template<typename ListIt>
int total_violated_time(const Problem& prob, ListIt route_first,
                        ListIt route_last);

bool satisfies_capacity(const Problem& prob, const Solution& sln);

bool satisfies_site_dependency(const Problem& prob, const Solution& sln);

bool satisfies_time_windows(const Problem& prob, const Solution& sln);
}  // namespace constraints
}  // namespace vrp
