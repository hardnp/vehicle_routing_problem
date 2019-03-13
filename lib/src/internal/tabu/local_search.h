#pragma once

#include "solution.h"

#include <cstdint>
#include <functional>
#include <tuple>

namespace vrp {
namespace tabu {
class LocalSearchMethods {
    const Problem& m_prob;

    using methods_t = std::vector<std::function<Solution(const Solution&)>>;
    // typedef Solution (*function_t)(const Solution&);
    // using methods_t = std::vector<function_t>;
    // using methods_t = std::vector<Solution (*)(const Solution&)>;
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
    Solution exchange(const Solution& sln);

    Solution relocate(const Solution& sln);

    Solution relocate_split(const Solution& sln);

    Solution two_opt(const Solution& sln);
};
}  // namespace tabu
}  // namespace vrp
