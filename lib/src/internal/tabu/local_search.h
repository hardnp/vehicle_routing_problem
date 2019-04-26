#pragma once

#include "solution.h"
#include "tabu_lists.h"

#include <cstdint>
#include <functional>
#include <string>
#include <tuple>

namespace vrp {
namespace tabu {
class LocalSearchMethods {
    const Problem& m_prob;

    using methods_t = std::vector<std::function<bool(Solution&, TabuLists&)>>;
    methods_t m_methods = methods_t(6);
    std::vector<double> m_best_values =
        std::vector<double>(6, std::numeric_limits<double>::max());

    double m_tw_penalty = 0.0;      ///< penalty for time windows violation
    bool m_can_violate_tw = false;  ///< flag to specify if TW can be violated
    bool m_enable_splits = false;   ///< flag to enable split delivery
    SplitInfo m_default_split_info = {};  ///< default split info

    bool m_explore_all_neighbourhoods =
        false;  ///< explore all solution, do not use "first improvement"
                ///< strategy

    // main local search heuristics. accessed via operator[]
    bool exchange(Solution& sln, TabuLists& lists, size_t method_id);
    bool relocate(Solution& sln, TabuLists& lists, size_t method_id);
    bool two_opt(Solution& sln, TabuLists& lists, size_t method_id);
    bool cross(Solution& sln, TabuLists& lists, size_t method_id);
    bool relocate_new_route(Solution& sln, TabuLists& lists, size_t method_id);
    bool relocate_split(Solution& sln, TabuLists& lists, size_t method_id);

public:
    LocalSearchMethods() = delete;
    LocalSearchMethods(const Problem& prob) noexcept;

    methods_t::iterator begin();
    methods_t::iterator end();

    methods_t::const_iterator cbegin() const;
    methods_t::const_iterator cend() const;

    size_t size() const;
    const methods_t::value_type& operator[](size_t i) const;
    std::string str(size_t i) const;

    // additional heuristics:
    void route_save(Solution& sln, size_t threshold);
    void intra_relocate(Solution& sln);
    void merge_splits(Solution& sln);

    // time windows management:
    void penalize_tw(double value);
    void violate_tw(bool value);
};
}  // namespace tabu
}  // namespace vrp
