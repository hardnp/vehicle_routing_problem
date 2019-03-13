#include "solution.h"

namespace vrp {
void Solution::update_times(const Problem& prob) {
    auto& sln = *this;

    sln.times.reserve(sln.routes.size());
    for (const auto& values : sln.routes) {
        sln.times.emplace_back(values.first, std::list<vrp::RoutePointTime>{});
    }
    // TODO: optimize
    static const auto at = [](const auto& list, size_t i) {
        if (list.size() <= i) {
            throw std::out_of_range("index out of list's range");
        }
        return *std::next(list.begin(), i);
    };
    const auto& customers = prob.customers;
    for (size_t ri = 0; ri < sln.routes.size(); ++ri) {
        const auto& route = sln.routes[ri].second;
        auto& time = sln.times[ri].second;
        int start_time = 0;
        for (size_t i = 0; i < route.size() - 1; ++i) {
            auto c = at(route, i);
            auto next_c = at(route, i + 1);

            RoutePointTime t;
            t.arrive = start_time;
            t.start = std::max(t.arrive, customers[c].hard_tw.first);
            t.finish = t.start + customers[c].service_time;

            start_time += t.finish + prob.times[c][next_c];

            time.emplace_back(std::move(t));
        }
        // last node is a depot
        time.emplace_back(start_time, start_time, start_time);
    }
}
}  // namespace vrp
