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

    for (const auto& vehicle_and_route : sln.routes) {
        const auto& route = vehicle_and_route.second;
        violated_time +=
            total_violated_time(prob, route.cbegin(), route.cend());
    }

    return violated_time;
}

template<typename ListIt>
int total_violated_time(const Problem& prob, ListIt first, ListIt last) {
    static_assert(
        std::is_same<size_t, std::decay_t<typename std::iterator_traits<
                                 ListIt>::value_type>>::value,
        "unexpected iterator value type");
    if (first == last) {
        throw std::runtime_error("unable to count violated time");
    }

    int violated_time = 0;
    auto next_first = std::next(first, 1);
    const auto& customers = prob.customers;

    int start_time = 0;
    for (; next_first != last; ++first, ++next_first) {
        auto c = customers[*first], next_c = customers[*next_first];

        // c.hard_tw[0] + c.service + distance(c, next_c) + next_c.service
        // <=
        // next_c.hard_tw[1]
        int spent_time_on_c =
            start_time + c.service_time + prob.costs[*first][*next_first];
        start_time = std::max(spent_time_on_c, next_c.hard_tw.first);
        spent_time_on_c += next_c.service_time;

        violated_time += std::max(0, spent_time_on_c - next_c.hard_tw.second);
    }

    return violated_time;
}

bool satisfies_capacity(const Problem& prob, const Solution& sln) {
    bool satisfies = true;
    const auto& customers = prob.customers;
    for (const auto& vehicle_and_route : sln.routes) {
        const auto& vehicle = prob.vehicles[vehicle_and_route.first];
        const auto& route = vehicle_and_route.second;
        satisfies &= (vehicle.capacity >=
                      std::accumulate(route.cbegin(), route.cend(),
                                      TransportationQuantity{},
                                      [&customers](const auto& init, size_t c) {
                                          return init + customers[c].demand;
                                      }));
    }
    return satisfies;
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

bool satisfies_time_windows(const Problem& prob, const Solution& sln) {
    return 0 == total_violated_time(prob, sln);
}
}  // namespace constraints
}  // namespace vrp
