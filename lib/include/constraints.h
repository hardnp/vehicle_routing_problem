#pragma once

#include "problem.h"
#include "solution.h"

#include <cassert>
#include <cmath>
#include <ostream>

namespace vrp {
namespace constraints {
int total_violated_time(const Problem& prob, const Solution& sln);

template<typename ListIt>
inline int total_violated_time(const Problem& prob, const SplitInfo& info,
                               ListIt first, ListIt last) {
    static_assert(
        std::is_same<size_t, std::decay_t<typename std::iterator_traits<
                                 ListIt>::value_type>>::value,
        "unexpected iterator value type");
    if (first == last) {
        throw std::runtime_error("unable to count violated time");
    }

    const auto service_time = [&info](const Customer& c) -> int {
        return static_cast<int>(std::ceil(info.at(c.id) * c.service_time));
    };

    int violated_time = 0;
    auto next_first = std::next(first);
    const auto& customers = prob.customers;

    int start_time = 0;
    for (; next_first != last; ++first, ++next_first) {
        const auto& c = customers[*first];
        const auto& next_c = customers[*next_first];
        assert(static_cast<size_t>(c.id) == *first);
        assert(static_cast<size_t>(next_c.id) == *next_first);

        // c.hard_tw[0] + c.service + distance(c, next_c) + W* + next_c.service
        // <=
        // next_c.hard_tw[1]
        //
        // * - W is the wait time if vehicle arrived to next_c earlier than
        //     next_c.hard_tw[0]
        int spent_time_on_c =
            start_time + service_time(c) + prob.costs[*first][*next_first];

        // include waiting for next_c TW start
        spent_time_on_c = std::max(spent_time_on_c, next_c.hard_tw.first);

        // new start time on next_c is spent_time_on_c
        start_time = spent_time_on_c;

        spent_time_on_c += service_time(next_c);

        violated_time += std::max(0, spent_time_on_c - next_c.hard_tw.second);
    }

    return violated_time;
}

TransportationQuantity total_violated_capacity(const Problem& prob,
                                               const Solution& sln);

template<typename ListIt>
TransportationQuantity
total_violated_capacity(const Problem& prob, TransportationQuantity cap,
                        const SplitInfo& info, ListIt first, ListIt last) {
    static_assert(
        std::is_same<size_t, std::decay_t<typename std::iterator_traits<
                                 ListIt>::value_type>>::value,
        "unexpected iterator value type");
    if (first == last) {
        throw std::runtime_error("unable to count violated capacity");
    }

    const auto& customers = prob.customers;
    for (; first != last; ++first) {
        const auto& c = customers[*first];
        assert(static_cast<size_t>(c.id) == *first);
        cap -= (c.demand * info.at(c.id));
    }

    TransportationQuantity violated_capacity = {};
    if (cap.volume < 0 || cap.weight < 0) {
        violated_capacity = -1 * cap;
    }
    return violated_capacity;
}

inline bool satisfies_capacity(const Problem& prob, const Solution& sln) {
    return total_violated_capacity(prob, sln) == 0;
}

bool satisfies_site_dependency(const Problem& prob, const Solution& sln);

inline bool satisfies_time_windows(const Problem& prob, const Solution& sln) {
    return total_violated_time(prob, sln) == 0;
}

bool satisfies_vehicle_uniqueness(const Problem& prob, const Solution& sln);

inline bool satisfies_routes_limit(const Problem& prob, const Solution& sln) {
    return prob.n_vehicles() >= sln.routes.size();
}

bool satisfies_customers_service(const Problem& prob, const Solution& sln);

bool satisfies_split_delivery(const Problem& prob, const Solution& sln);

bool satisfies_depots(const Problem& prob, const Solution& sln);

/// Returns whether given solution satisfies all constraints, optionally writing
/// unsatisfied constraints to err stream (if err is not nullptr)
bool satisfies_all(const Problem& prob, const Solution& sln,
                   std::ostream* err = nullptr);
}  // namespace constraints
}  // namespace vrp
