#pragma once

#include "problem.h"
#include "solution.h"

namespace vrp {
namespace constraints {
int total_violated_time(const Problem& prob, const Solution& sln);

template<typename ListIt>
inline int total_violated_time(const Problem& prob, ListIt first, ListIt last) {
    static_assert(
        std::is_same<size_t, std::decay_t<typename std::iterator_traits<
                                 ListIt>::value_type>>::value,
        "unexpected iterator value type");
    if (first == last) {
        throw std::runtime_error("unable to count violated time");
    }

    int violated_time = 0;
    auto next_first = std::next(first);
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

TransportationQuantity total_violated_capacity(const Problem& prob,
                                               const Solution& sln);

template<typename ListIt>
TransportationQuantity total_violated_capacity(const Problem& prob,
                                               TransportationQuantity cap,
                                               ListIt first, ListIt last) {
    static_assert(
        std::is_same<size_t, std::decay_t<typename std::iterator_traits<
                                 ListIt>::value_type>>::value,
        "unexpected iterator value type");
    if (first == last) {
        throw std::runtime_error("unable to count violated capacity");
    }

    const auto& customers = prob.customers;
    for (; first != last; ++first) {
        cap -= customers[*first].demand;
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

bool satisfies_all(const Problem& prob, const Solution& sln);
}  // namespace constraints
}  // namespace vrp
