#pragma once

#include "problem.h"
#include "route_point.h"
#include "vehicle.h"

#include <list>
#include <utility>
#include <vector>

namespace vrp {
/// Solution representation
class Solution {
public:
    using VehicleIndex = size_t;   ///< vehicle index in problem's vector
                                   /// index != id
    using CustomerIndex = size_t;  ///< customer index in problem's vector
                                   /// index != id
    std::vector<std::pair<VehicleIndex, std::list<CustomerIndex>>> routes;

    std::vector<std::pair<VehicleIndex, std::list<RoutePointTime>>> times;
};
}  // namespace vrp
