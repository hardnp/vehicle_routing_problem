#pragma once

#include "problem.h"
#include "route_point.h"
#include "vehicle.h"

#include <list>
#include <unordered_set>
#include <utility>
#include <vector>

namespace vrp {
/// Solution representation
class Solution {
public:
    using VehicleIndex = size_t;   ///< vehicle index in problem's vector
    using CustomerIndex = size_t;  ///< customer index in problem's vector
    using RouteType = std::list<CustomerIndex>;
    std::vector<std::pair<VehicleIndex, RouteType>> routes;

    std::vector<std::pair<VehicleIndex, std::list<RoutePointTime>>> times;

    std::vector<std::pair<size_t, size_t>>
        customer_owners;  ///< specifies which route
                          ///< each customer belongs to and what index customer
                          ///< has

    std::unordered_set<VehicleIndex>
        used_vehicles;  ///< vehicles used by solution

    void update_times(const Problem& prob);

    void update_customer_owners(const Problem& prob);
    void update_customer_owners(const Problem& prob, size_t route_index,
                                size_t first_customer_index = 0);

    void update_used_vehicles();

    bool operator==(const Solution& other) const;

    explicit operator bool();
};
}  // namespace vrp
