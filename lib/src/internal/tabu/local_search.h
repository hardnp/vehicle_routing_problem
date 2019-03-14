#pragma once

#include "solution.h"

#include <cstdint>
#include <functional>
#include <tuple>

namespace vrp {
namespace tabu {
class LocalSearchMethods {
    const Problem& m_prob;

    using methods_t = std::vector<std::function<void(Solution&)>>;
    methods_t m_methods = methods_t(4);

public:
    LocalSearchMethods() = delete;
    LocalSearchMethods(const Problem& prob) noexcept;

    methods_t::iterator begin();
    methods_t::iterator end();

    methods_t::const_iterator cbegin() const;
    methods_t::const_iterator cend() const;

    size_t size() const;
    const methods_t::value_type& operator[](size_t i) const;

    // TODO: return tabu move as well
    void exchange(Solution& sln);
    void relocate(Solution& sln);
    void relocate_split(Solution& sln);
    void two_opt(Solution& sln);
};
}  // namespace tabu
}  // namespace vrp
