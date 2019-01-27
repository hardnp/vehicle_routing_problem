#include "cluster_first_route_second.h"

#include <algorithm>
#include <cassert>
#include <random>
#include <numeric>
#include <utility>

#define ILOUSESTL
using namespace std;
#include "ilcplex/ilocplex.h"

/// Specify that the variable x is unused in the code
#define UNUSED(x) (void)x

namespace vrp {
namespace detail {

namespace {
using CustomerIndex = size_t;  // customer id

/// Get uniform random integer in closed interval [min, max]
size_t get_random(size_t min, size_t max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    gen.seed(7);
    static std::uniform_int_distribution<size_t> dist(min, max);
    return dist(gen);
}

/// Return most similar seed with regards to j
CustomerIndex most_similar_seed(const Problem& prob,
    const std::vector<CustomerIndex>& seeds, CustomerIndex j) {
    return *std::min_element(
        seeds.cbegin(), seeds.cend(),
        [j, &prob] (CustomerIndex a, CustomerIndex b) {
            return prob.times[a][j] < prob.times[b][j];
    });
}

/// Return most similar and second most similar seeds with regards to j
std::pair<CustomerIndex, CustomerIndex> two_most_similar_seeds(
    const Problem& prob, const std::vector<CustomerIndex>& seeds,
    CustomerIndex j) {
    auto d = [j, &prob] (CustomerIndex i) {  // dissimilarity function
        return prob.times[i][j];
    };
    CustomerIndex most_similar = 0;
    CustomerIndex second_most_similar = 1;
    if (d(most_similar) > d(second_most_similar)) {
        std::swap(most_similar, second_most_similar);
    }
    for (size_t i = 2; i < seeds.size(); ++i) {
        if (d(most_similar) > d(seeds[i])) {
            second_most_similar = most_similar;
            most_similar = i;
        } else if (d(second_most_similar) > d(seeds[i])) {
            second_most_similar = i;
        }
    }
    return std::make_pair(most_similar, second_most_similar);
}

/// Partitioning Around Medoids (PAM) method
std::vector<CustomerIndex> find_seeds(const Problem& prob) {
    const auto K = prob.n_vehicles();  // number of seeds
    const auto Q = double(std::accumulate(
        prob.customers.cbegin(), prob.customers.cend(), 0,
        [] (int running_cost, const Customer& c)
        { return running_cost + c.demand; })) / K;  // total (average) demand
                                                    // per seed
    UNUSED(Q);
    auto n_customers = prob.n_customers();
    std::vector<bool> is_customer_selected(n_customers, false);
    std::vector<CustomerIndex> seeds = {};
    seeds.reserve(K);
    // BUILD phase
    {
        for (size_t it = seeds.size(); it < K; ++it) {
            int max_total_gain = 0;
            auto best_candidate = std::numeric_limits<CustomerIndex>::max();
            for (CustomerIndex i = 1; i < n_customers; ++i) {
                if (is_customer_selected[i]) continue;  // skip selected i
                int C_i = 0;
                for (CustomerIndex j = 1; j < n_customers; ++j) {
                    if (is_customer_selected[j]) continue;  // skip selected j
                    auto most_similar = most_similar_seed(prob, seeds, j);
                    auto D_j = prob.times[most_similar][j];
                    // calculate d(i, j) - dissimilarity between i and j
                    auto d_ij = prob.times[i][j];
                    // calculate C(i, j) - total contribution of j to select i
                    // add it to C(i) - total gain of i
                    C_i += std::max(0, D_j - d_ij);
                }
                if (C_i > max_total_gain) {
                    max_total_gain = C_i;
                    best_candidate = i;
                }
            }
            is_customer_selected[best_candidate] = true;
            seeds.push_back(best_candidate);
        }
    }
    assert(seeds.size() == K);
    // SWAP phase
    {
        bool stop = false;
        // FIXME: the following is a dirty WORKAROUND to prevent the phase from
        // swapping the same nodes back and forth (e.g. 84 -> 85 and 85 -> 84)
        std::pair<CustomerIndex, CustomerIndex> last_time_swapped =
            std::make_pair(std::numeric_limits<CustomerIndex>::max(),
                std::numeric_limits<CustomerIndex>::max());
        auto is_cycle = [] (std::pair<CustomerIndex, CustomerIndex> a,
            std::pair<CustomerIndex, CustomerIndex> b) {
            // check if a == b
            return (a.first == b.first && a.second == b.second) ||
                (a.second == b.first && a.first == b.second);
        };
        while(!stop) {
            int min_T = std::numeric_limits<int>::max();
            auto best_i = std::numeric_limits<CustomerIndex>::max(),
                best_h = std::numeric_limits<CustomerIndex>::max();
            for (CustomerIndex i : seeds) {
                for (CustomerIndex h = 0; h < n_customers; ++h) {
                    if (is_customer_selected[h]) continue;  // skip selected h
                    int T_ih = 0;
                    for (CustomerIndex j = 0; j < n_customers; ++j) {
                        if (is_customer_selected[j] || j == h) continue;
                        auto d_ij = prob.times[i][j];  // calculate d(i, j)
                        auto d_hj = prob.times[h][j];  // calculate d(h, j)
                        auto two_most_similar = two_most_similar_seeds(
                            prob, seeds, j);
                        auto most_similar = two_most_similar.first;
                        auto D_j = prob.times[most_similar][j];
                        auto E_j = prob.times[two_most_similar.second][j];
                        // calculate C(i, h, j) - total contribution of j to
                        // swap i with h (make h selected instead of i)
                        // add it to C(i, h) - total gain of pair (i, h)
                        int C_ihj = 0;
                        // a. j is more distant from both i and h than from one
                        // of the other representative objects
                        if (d_ij > D_j && d_hj > D_j) {
                            C_ihj = 0;
                        }
                        // b. d(i, j) == D(j)
                        else if (d_ij == D_j) {
                            // b1. j is closer to h than to the second closest
                            // representative object
                            if (E_j > d_hj) {
                                C_ihj = d_hj - d_ij;
                            }
                            // b2. j is at least as distant from h than from
                            // second closest representative object
                            else {
                                C_ihj = E_j - D_j;
                            }
                        }
                        // c. j is closer to h than to any representative object
                        else if (D_j > d_hj) {
                            C_ihj = d_hj - D_j;
                        }
                        T_ih += C_ihj;
                    }
                    // minimize T(i, h)
                    if (min_T > T_ih) {
                        min_T = T_ih;
                        best_i = i;
                        best_h = h;
                    }
                }
            }
            auto swap_cadidate = std::make_pair(best_i, best_h);
            // perform swap if min(T(i, j)) < 0 AND this swap did not happen
            // before (workaround to prevent cyclic swapping)
            if (min_T < 0 && !is_cycle(last_time_swapped, swap_cadidate)) {
                last_time_swapped = std::make_pair(best_i, best_h);
                is_customer_selected[best_i] = false;
                is_customer_selected[best_h] = true;
                auto iter = std::find(seeds.cbegin(), seeds.cend(), best_i);
                seeds[std::distance(seeds.cbegin(), iter)] = best_h;
            } else {
                stop = true;
            }
        }
    }
    return seeds;
}
}  // anonymous

std::vector<Solution> cfrs_impl(const Problem& prob, size_t count) {
    auto seeds = find_seeds(prob);
    UNUSED(seeds);
    return {};
}

}  // detail
}  // vrp
