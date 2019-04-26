#include "constraints.h"

#include <algorithm>
#include <iterator>
#include <numeric>
#include <type_traits>

namespace vrp {
namespace constraints {
namespace {
// return if customer and vehicle are site-dependent. true if vehicle can
// deliver to customer, false otherwise
bool site_dependent(const Problem& prob, size_t vehicle, size_t customer) {
    if (customer >= prob.allowed_vehicles_size()) {
        throw std::out_of_range("customer >= allowed vehicles size");
    }
    const auto& allowed = prob.allowed_vehicles(customer);
    if (vehicle >= allowed.size()) {
        throw std::out_of_range("vehicle >= allowed vehicles(customer) size");
    }
    return allowed[vehicle];
}
}  // namespace

int total_violated_time(const Problem& prob, const Solution& sln) {
    int violated_time = 0;

    for (size_t t = 0, size = sln.routes.size(); t < size; ++t) {
        const auto& route = sln.routes[t].second;
        violated_time += total_violated_time(prob, sln.route_splits.at(t),
                                             route.cbegin(), route.cend());
    }

    return violated_time;
}

TransportationQuantity total_violated_capacity(const Problem& prob,
                                               const Solution& sln) {
    TransportationQuantity violated_capacity = {};

    const auto& vehicles = prob.vehicles;
    for (size_t t = 0, size = sln.routes.size(); t < size; ++t) {
        const auto& vehicle_and_route = sln.routes[t];
        size_t v = vehicle_and_route.first;
        const auto& route = vehicle_and_route.second;
        violated_capacity += total_violated_capacity(
            prob, vehicles[v].capacity, sln.route_splits.at(t), route.cbegin(),
            route.cend());
    }

    return violated_capacity;
}

bool satisfies_site_dependency(const Problem& prob, const Solution& sln) {
    bool satisfies = true;
    for (const auto& vehicle_and_route : sln.routes) {
        const size_t v = vehicle_and_route.first;
        const auto& route = vehicle_and_route.second;
        satisfies &=
            std::all_of(route.cbegin(), route.cend(), [&v, &prob](size_t c) {
                return site_dependent(prob, v, c);
            });
    }
    return satisfies;
}

bool satisfies_all(const Problem& prob, const Solution& sln) {
    static std::vector<bool (*)(const Problem&, const Solution&)> checkers = {
        satisfies_site_dependency, satisfies_capacity, satisfies_time_windows};

    bool satisfied = true;
    for (const auto& checker : checkers) {
        satisfied &= checker(prob, sln);
    }

    return satisfied;
}
}  // namespace constraints
}  // namespace vrp
