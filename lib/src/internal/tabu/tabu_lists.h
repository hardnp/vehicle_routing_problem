#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iterator>
#include <set>
#include <stack>
#include <type_traits>
#include <unordered_set>
#include <utility>

// can be overriden from the outside
#ifndef TABU_TENURE
#define TABU_TENURE 15
#define PRESERVE_TENURE 7
#endif

namespace vrp {
namespace tabu {
#define USE_HASH_SET 0

namespace detail {
template<typename T, int Tenure> struct Entry : public T {
    // T value;
    int count = Tenure;

    Entry() = delete;
    inline Entry(const T& v) : T(v) {}
    inline Entry(T&& v) : T(std::move(v)) {}
    // bool operator==(const Entry& other) const { return value == other.value;
    // } bool operator<(const Entry& other) const { return value < other.value;
    // } bool operator==(const T& other_value) const { return value ==
    // other_value; }
};

#if USE_HASH_SET
template<typename U, int Tenure = 0> struct Hash {};

template<int Tenure> struct Hash<std::pair<size_t, size_t>, Tenure> {
    inline size_t
    operator()(const Entry<std::pair<size_t, size_t>, Tenure>& e) const
        noexcept {
        return (e.first << 16) ^ e.second;
    }
};

template<int Tenure> struct Hash<size_t, Tenure> {
    inline size_t operator()(const Entry<size_t, Tenure>& e) const noexcept {
        return e;
    }
};
#endif
}  // namespace detail

template<typename T, int Tenure> class TabuList {
#if USE_HASH_SET
    using set_t =
        std::unordered_set<detail::Entry<T, Tenure>, detail::Hash<T, Tenure>>;
#else
    using set_t = std::set<detail::Entry<T, Tenure>>;
#endif
    set_t entries;

    inline set_t diff(const set_t& lhs, const set_t& rhs) {
        set_t result;
        // TODO: is it only me or std::set_difference doesn't return __unique__
        // values but only found in first of two sets?
        std::set_difference(rhs.cbegin(), rhs.cend(), lhs.cbegin(), lhs.cend(),
                            std::inserter(result, result.end()));
        return result;
    }

public:
    TabuList() = default;
    ~TabuList() = default;
    TabuList(const TabuList& other) = default;
    TabuList(TabuList&& other) = default;
    TabuList& operator=(const TabuList& other) {
        // only add entries that do not exist in tabu list
        auto new_entries = diff(entries, other.entries);
        std::copy(std::make_move_iterator(new_entries.begin()),
                  std::make_move_iterator(new_entries.end()),
                  std::inserter(entries, entries.end()));
        return *this;
    }
    TabuList& operator=(TabuList&& other) {
        // only add entries that do not exist in tabu list
        auto new_entries = diff(entries, other.entries);
        std::copy(std::make_move_iterator(new_entries.begin()),
                  std::make_move_iterator(new_entries.end()),
                  std::inserter(entries, entries.end()));
        return *this;
    }

    inline void decrement() {
        // decrement all, erase expired tabu entries
        set_t non_expired_entries = {};
        for (auto entry : entries) {
            entry.count--;
            if (entry.count > 0) {
                non_expired_entries.emplace(entry);
            }
        }
        entries = std::move(non_expired_entries);
    }

    inline void clear() { entries.clear(); }

    template<typename... Args> void emplace(Args&&... args) {
        entries.emplace(T(std::forward<Args>(args)...));
    }

    template<typename... Args> bool has(Args&&... args) {
        return std::find(entries.cbegin(), entries.cend(),
                         T(std::forward<Args>(args)...)) != entries.cend();
    }

    set_t& all() { return entries; }
    const set_t& all() const { return entries; }
};  // namespace tabu

class TabuLists {
    using tabu_list_t = TabuList<std::pair<size_t, size_t>, TABU_TENURE>;
    using preserve_list_t = tabu_list_t;

public:
    // tabu lists. each entry depends on a used heuristic
    // pair of customer && route
    tabu_list_t exchange = {};
    // pair of customer && route
    tabu_list_t relocate = {};
    // pair of customers
    tabu_list_t two_opt = {};
    // pair of customers
    tabu_list_t cross = {};
    // pair of customer && route
    tabu_list_t relocate_split = {};
    // pair of customer && route
    tabu_list_t relocate_new_route = {};

    // preserve lists: data stored is similar to tabu lists, but the purpose is
    // to preserve node at the place where it belongs. in a nutshell, preserve
    // list is the opposite of tabu list
    preserve_list_t pr_exchange = {};
    preserve_list_t pr_relocate = {};
    preserve_list_t pr_two_opt = {};
    preserve_list_t pr_cross = {};
    preserve_list_t pr_relocate_split = {};
    preserve_list_t pr_relocate_new_route = {};

    TabuLists& operator--() {
        exchange.decrement();
        relocate.decrement();
        two_opt.decrement();
        cross.decrement();
        relocate_split.decrement();
        relocate_new_route.decrement();

        pr_exchange.decrement();
        pr_relocate.decrement();
        pr_two_opt.decrement();
        pr_cross.decrement();
        pr_relocate_split.decrement();
        pr_relocate_new_route.decrement();

        return *this;
    }

    void clear() {
        exchange.clear();
        relocate.clear();
        two_opt.clear();
        cross.clear();
        relocate_split.clear();
        relocate_new_route.clear();

        pr_exchange.clear();
        pr_relocate.clear();
        pr_two_opt.clear();
        pr_cross.clear();
        pr_relocate_split.clear();
        pr_relocate_new_route.clear();
    }
};

#undef USE_HASH_SET
}  // namespace tabu
}  // namespace vrp
