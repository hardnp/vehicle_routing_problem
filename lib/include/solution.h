#pragma once

#include "vehicle.h"
#include "problem.h"
#include "route_point.h"

#include <utility>
#include <vector>

namespace vrp {
/// Solution representation
class Solution {
public:
	using VehicleIndex = size_t;  ///< vehicle index(!= id) in problem's vector

	std::vector<std::pair<VehicleIndex, std::vector<RoutePoint>>> routes;
};
}  // vrp
