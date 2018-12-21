#include "cluster_first_route_second.h"

#include <algorithm>
#include <random>
#include <numeric>

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

/// Partitioning Around Medoids (PAM) method
std::vector<CustomerIndex> find_seeds(const Problem& prob) {
    const auto K = prob.n_vehicles();  // number of seeds
    const auto Q = double(std::accumulate(
        prob.customers.cbegin(), prob.customers.cend(), 0,
        [] (int running_cost, const Customer& c)
        { return running_cost + c.demand; })) / K;  // total (average) demand
                                                    // per seed
    auto n_customers = prob.n_customers();
    std::vector<bool> is_customer_selected(n_customers, false);
    std::vector<CustomerIndex> seeds = {};
    seeds.reserve(K);
    // BUILD phase
    {
        for (size_t it = seeds.size(); it < K; ++it) {
            std::vector<double> total_gains(n_customers, 0.0);
            double max_total_gain = 0.0;
            auto best_candidate = std::numeric_limits<CustomerIndex>::max();
            for (CustomerIndex i = 1; i < n_customers; ++i) {
                if (is_customer_selected[i]) continue;  // skip selected i
                double C_i = 0.0;
                for (CustomerIndex j = 1; j < n_customers; ++j) {
                    if (is_customer_selected[j]) continue;  // skip selected j
                    // calculate D(j) - dissimilarity of j w.r.t. selected nodes
                    CustomerIndex most_similar = *std::min_element(
                        seeds.cbegin(), seeds.cend(),
                        [j, &prob] (CustomerIndex a, CustomerIndex b) {
                            return prob.costs[a][j] < prob.costs[b][j];
                        });
                    auto D_j = prob.costs[most_similar][j];
                    // calculate d(i, j) - dissimilarity between i and j
                    auto d_ij = prob.costs[i][j];
                    // calculate C(i, j) - total contribution of j to select i
                    // add it to C(i) - total gain of i
                    C_i += std::max(0.0, D_j - d_ij);
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
    // SWAP phase
    return seeds;
}
}  // anonymous

std::vector<Solution> cluster_first_route_second(const Problem& prob,
    size_t count) {
    auto seeds = find_seeds(prob);
    return {};
}
}  // detail
}  // vrp
