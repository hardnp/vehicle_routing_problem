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
        double d = 1.0;  ///< ratio as a double value
        inline operator double() const noexcept { return d; }
        inline Ratio& operator=(double v) {
            d = v;
            return *this;
        }
        inline Ratio& operator+=(const Ratio& other) {
            d += other.d;
            return *this;
        }
        inline Ratio& operator-=(const Ratio& other) {
            d -= other.d;
            return *this;
        }
    };
    std::unordered_map<size_t, Ratio> split_info;  ///< index mapped to ratio
    inline bool has(size_t i) const noexcept {
        return split_info.cend() != split_info.find(i);
    }
    inline bool has_any(const std::vector<size_t>& is) const noexcept {
        return std::any_of(is.cbegin(), is.cend(),
                           [this](size_t i) { return has(i); });
    }
    inline const Ratio& at(size_t i) const { return split_info.at(i); }
    inline bool empty() const noexcept { return split_info.empty(); }
};

void transfer_split_entry(bool enable_splits, SplitInfo& src, SplitInfo& dst,
                          size_t key);
template<typename ListIt>
inline void transfer_split_entry(bool enable_splits, SplitInfo& src,
                                 SplitInfo& dst, ListIt first, ListIt last) {
    // do nothing if splits are disabled
    if (!enable_splits) {
        return;
    }

    for (; first != last; ++first) {
        transfer_split_entry(enable_splits, src, dst, *first);
    }
}

/// Solution representation
class Solution {
public:
    using VehicleIndex = size_t;   ///< vehicle index in problem's vector
    using CustomerIndex = size_t;  ///< customer index in problem's vector
    using RouteType = std::list<CustomerIndex>;
    std::vector<std::pair<VehicleIndex, RouteType>> routes;

    std::vector<std::pair<VehicleIndex, std::list<RoutePointTime>>> times;

    std::vector<std::unordered_map<size_t, size_t>>
        customer_owners;  ///< specifies which route
                          ///< each customer belongs to and what index customer
                          ///< has

    std::unordered_set<VehicleIndex>
        used_vehicles;  ///< vehicles used by solution

    // TODO: this is extra information. It may be better to put SplitInfo into
    //       routes
    std::vector<SplitInfo> route_splits;  ///< split info for each route

    void update_times(const Problem& prob);

    void update_customer_owners(const Problem& prob);
    void update_customer_owners(const Problem& prob, size_t route_index,
                                size_t first_customer_index = 0);

    void update_used_vehicles();

    bool operator==(const Solution& other) const;

    inline operator bool() const noexcept { return this->routes.empty(); }
};
}  // namespace vrp
