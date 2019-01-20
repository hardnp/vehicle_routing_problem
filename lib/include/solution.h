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
    using CustomerIndex = size_t;  ///< customer index(!= id) in problem's vector

    std::vector<std::pair<VehicleIndex, std::vector<CustomerIndex>>> routes;

    std::vector<std::vector<std::pair<int, RoutePointTime>>> times;
};
}  // vrp
