#pragma once

#include "solution.h"
#include "tabu_lists.h"

#include <cstdint>
#include <functional>
#include <tuple>

namespace vrp {
namespace tabu {
class LocalSearchMethods {
    const Problem& m_prob;

    using methods_t = std::vector<std::function<void(Solution&, TabuLists&)>>;
    methods_t m_methods = methods_t(4);

    double m_tw_penalty = 0.0;  ///< penalty for time windows violation

public:
    LocalSearchMethods() = delete;
    LocalSearchMethods(const Problem& prob) noexcept;

    methods_t::iterator begin();
    methods_t::iterator end();

    methods_t::const_iterator cbegin() const;
    methods_t::const_iterator cend() const;

    size_t size() const;
    const methods_t::value_type& operator[](size_t i) const;

    // main local search heuristics:
    void exchange(Solution& sln, TabuLists& lists);
    void relocate(Solution& sln, TabuLists& lists);
    void relocate_split(Solution& sln, TabuLists& lists);
    void two_opt(Solution& sln, TabuLists& lists);

    // additional heuristics:
    void route_save(Solution& sln, size_t threshold);
    void intra_relocate(Solution& sln);

    // time windows management:
    void set_tw_penalty(double value);
};
}  // namespace tabu
}  // namespace vrp
