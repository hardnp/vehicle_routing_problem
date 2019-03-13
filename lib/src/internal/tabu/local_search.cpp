#include "local_search.h"

#include <stdexcept>

namespace vrp {
namespace tabu {
LocalSearchMethods::LocalSearchMethods(const Problem& prob) noexcept
    : m_prob(prob) {
    m_methods[0] =
        std::bind(&LocalSearchMethods::relocate, this, std::placeholders::_1);
    m_methods[1] = std::bind(&LocalSearchMethods::relocate_split, this,
                             std::placeholders::_1);
    m_methods[2] =
        std::bind(&LocalSearchMethods::exchange, this, std::placeholders::_1);
    m_methods[3] =
        std::bind(&LocalSearchMethods::two_opt, this, std::placeholders::_1);
}

LocalSearchMethods::methods_t::iterator LocalSearchMethods::begin() {
    return m_methods.begin();
}

LocalSearchMethods::methods_t::iterator LocalSearchMethods::end() {
    return m_methods.end();
}

LocalSearchMethods::methods_t::const_iterator
LocalSearchMethods::cbegin() const {
    return m_methods.cbegin();
}

LocalSearchMethods::methods_t::const_iterator LocalSearchMethods::cend() const {
    return m_methods.cend();
}

size_t LocalSearchMethods::size() const { return m_methods.size(); }

const LocalSearchMethods::methods_t::value_type& LocalSearchMethods::
operator[](size_t i) const {
    if (i >= size()) {
        throw std::out_of_range("index >= size");
    }
    return m_methods[i];
}

Solution LocalSearchMethods::relocate(const Solution& sln) { return sln; }

Solution LocalSearchMethods::relocate_split(const Solution& sln) { return sln; }

Solution LocalSearchMethods::exchange(const Solution& sln) { return sln; }

Solution LocalSearchMethods::two_opt(const Solution& sln) { return sln; }
}  // namespace tabu
}  // namespace vrp
