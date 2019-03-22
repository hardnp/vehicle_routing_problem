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

bool satisfies_capacity(const Problem& prob, const Solution& sln);

bool satisfies_site_dependency(const Problem& prob, const Solution& sln);

inline bool satisfies_time_windows(const Problem& prob, const Solution& sln) {
    return 0 == total_violated_time(prob, sln);
}
}  // namespace constraints
}  // namespace vrp
