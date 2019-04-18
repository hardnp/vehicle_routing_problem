#pragma once

#include "problem.h"
#include "route_point.h"
#include "vehicle.h"

#include <list>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace vrp {
/// Split info representation
struct SplitInfo {
    // TODO: decide upon proper design and usage
    struct Ratio {
        double d = 0.0;  ///< ratio as a double value
        // int i = 0.0;     ///< ratio as an integral value (a part of quantity)
        inline operator double() const noexcept { return d; }
        inline Ratio& operator=(double v) {
            d = v;
            return *this;
        }
    };
    std::unordered_map<size_t, Ratio> split_info;  ///< index mapped to ratio
    inline bool is_split() const noexcept { return split_info.size() > 1; }
    inline bool has(size_t i) const noexcept {
        return split_info.cend() !=
               std::find_if(split_info.cbegin(), split_info.cend(),
                            [i](const auto& p) { return p.first == i; });
    }
};

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

    std::vector<SplitInfo> customer_splits;  ///< split info for each customer

    void update_times(const Problem& prob);

    void update_customer_owners(const Problem& prob);
    void update_customer_owners(const Problem& prob, size_t route_index,
                                size_t first_customer_index = 0);

    void update_used_vehicles();

    bool operator==(const Solution& other) const;

    inline operator bool() const noexcept { return this->routes.empty(); }
};
}  // namespace vrp
