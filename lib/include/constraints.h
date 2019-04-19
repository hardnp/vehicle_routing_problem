#pragma once

#include "problem.h"
#include "solution.h"

#include <cassert>

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

    // TODO: is round() good enough here?
    const auto service_time = [&info](const Customer& c) -> int {
        // TODO: unordered_map::at() is sufficient?
        return static_cast<int>(std::round(info.at(c.id) * c.service_time));
    };

    int violated_time = 0;
    auto next_first = std::next(first);
    const auto& customers = prob.customers;

    int start_time = 0;
    for (; next_first != last; ++first, ++next_first) {
        const auto& c = customers[*first];
        const auto& next_c = customers[*next_first];
        assert(static_cast<size_t>(c.id) == *first);

        // c.hard_tw[0] + c.service + distance(c, next_c) + next_c.service
        // <=
        // next_c.hard_tw[1]
        int spent_time_on_c =
            start_time + service_time(c) + prob.costs[*first][*next_first];
        start_time = std::max(spent_time_on_c, next_c.hard_tw.first);
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

bool satisfies_all(const Problem& prob, const Solution& sln);
}  // namespace constraints
}  // namespace vrp
