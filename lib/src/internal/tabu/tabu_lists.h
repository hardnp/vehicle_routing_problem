#pragma once

#include <algorithm>
#include <cstdint>
#include <set>
#include <stack>
#include <unordered_set>
#include <utility>

// can be overriden from the outside
#ifndef TABU_TENURE
#define TABU_TENURE 15
#endif

namespace vrp {
namespace tabu {

// TODO: will this work for asymmetric cases: (customer, route) pairs?
#define USE_HASH_SET 0

template<typename T, int Tenure = 0> class TabuList {
    struct Hash;
    struct Entry {
        T value;
        int count = Tenure;
        Hash f;

        Entry() = delete;
        Entry(const T& v) : value(v) {}
        bool operator==(const Entry& other) const {
            return f(value) == f(other.value);
        }
        bool operator<(const Entry& other) const {
            return f(value) < f(other.value);
        }
        bool operator==(const T& other_value) const {
            return f(value) == f(other_value);
        }
    };

    struct Hash {
#if USE_HASH_SET
        inline size_t operator()(const Entry& e) const {
            return e.value.first ^ e.value.second;
        }
#else
        inline T operator()(const Entry& e) const { return e.value; }
#endif
    };

#if USE_HASH_SET
    using set_t = std::unordered_set<Entry, Hash>;
#else
    using set_t = std::set<Entry>;
#endif

    set_t entries;

    inline set_t diff(const set_t& lhs, const set_t& rhs) {
        set_t result;
        // TODO: is it only me or std::set_difference doesn't return __unique__
        // values only found in first of two sets?
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

public:
    // pair of customer && route
    tabu_list_t exchange = {};
    // pair of customer && route
    tabu_list_t relocate = {};
    // pair of customer && route
    tabu_list_t relocate_split = {};
    // pair of customers
    tabu_list_t two_opt = {};

    TabuLists& operator--() {
        exchange.decrement();
        relocate.decrement();
        relocate_split.decrement();
        two_opt.decrement();
        return *this;
    }
};
}  // namespace tabu
}  // namespace vrp
