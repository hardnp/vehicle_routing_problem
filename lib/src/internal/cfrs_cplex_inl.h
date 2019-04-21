#include "constraints.h"
#include "logging.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <list>
#include <numeric>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#define ILOUSESTL
using namespace std;
#include "ilcplex/ilocplex.h"

namespace vrp {
namespace detail {

namespace {
inline int volume(const TransportationQuantity& q) { return q.volume; }
inline int weight(const TransportationQuantity& q) { return q.weight; }

template<typename AttribAccessor>
int total(AttribAccessor accessor, const std::vector<Vehicle>& vehicles,
          const std::vector<size_t>& indices) {
    return std::accumulate(indices.cbegin(), indices.cend(), 0,
                           [&](int val, size_t i) {
                               return val + accessor(vehicles[i].capacity);
                           });
}

/// Cost of assigning customer i to seed
inline double assignment_cost(const Problem& prob, size_t seed, size_t i) {
    const auto& c = prob.costs;
    return c[0][i] + c[i][seed] - c[0][seed];
}

/// Min of costs of assigning customer i to each seed in seeds
double assignment_cost(const Problem& prob, const std::vector<size_t>& seeds,
                       size_t i) {
    const auto size = seeds.size();
    std::vector<double> costs(size);
    for (size_t s = 0; s < size; ++s) {
        costs[s] = assignment_cost(prob, seeds[s], i);
    }
    return *std::min_element(costs.cbegin(), costs.cend());
}

class Heuristic;

std::unordered_map<size_t, std::vector<size_t>> select_seeds(const Heuristic&);

constexpr const double SPLIT_THR = 0.25;

/// Heuristic class that solves the relaxed 0-1 Integer Problem
class Heuristic {
    const Problem& m_prob;

    IloEnv m_env;
    std::vector<IloNumVarArray> m_x;  ///< x[i][t]: 1 if customer i is assigned
                                      /// to type t. index is customer
    std::vector<IloIntVarArray> m_y;  ///< y[i][t]: internal value that forces
                                      ///< x[i][t] to be an integer or float
    IloModel m_model = IloModel(m_env);
    IloCplex m_algo = IloCplex(m_model);
    IloObjective m_objective;
    IloExprArray m_capacity_fractions;
    double m_alpha_v = 0.5, m_alpha_w = 0.5;  ///< volume/weight coefficients
                                              /// to balance zeroed values

    inline void add_zero_constraint(size_t i, IloInt t) {
        IloExpr zero_expr(m_env);
        zero_expr = m_x[i][t];
        m_model.add(IloConstraint(zero_expr == 0));
    }

    /// Cost of assigning customer i to seed s
    inline double assignment_cost(size_t seed, size_t i) {
        return ::vrp::detail::assignment_cost(m_prob, seed, i);
    }

    /// Min of costs of assigning customer i to each seed in seeds
    inline double assignment_cost(const std::vector<size_t>& seeds, size_t i) {
        return ::vrp::detail::assignment_cost(m_prob, seeds, i);
    }

public:
    inline IloEnv& env() { return m_env; }
    inline std::vector<IloNumVarArray>& X() { return m_x; }
    inline IloModel& model() { return m_model; }
    inline IloCplex& algo() { return m_algo; }
    inline IloObjective& objective() { return m_objective; }
    inline const Problem& prob() const { return m_prob; }

    /// Reference: A Computational Study of a New Heuristic for the
    /// Site-Dependent Vehicle Routing Problem. Chao, Golden, Wasil. 1998
    Heuristic(const Problem& prob) : m_prob(prob) {
#ifdef NDEBUG  // release mode
        m_algo.setOut(m_env.getNullStream());
#endif

        const auto n_customers = prob.n_customers();
        const auto n_vehicles = prob.n_vehicles();
        const auto& types = prob.vehicle_types();
        const auto types_size = types.size();

        m_x.reserve(n_customers);
        for (size_t i = 1; i < n_customers; ++i) {
            m_x.emplace_back(IloNumVarArray(m_env, types_size, 0.0, 1.0));
        }

        m_y.reserve(n_customers);
        for (size_t i = 1; i < n_customers; ++i) {
            m_y.emplace_back(IloIntVarArray(m_env, types_size, 0, 1));
        }

        {
            // TODO: too many constraints??

            // zero-out variables that correspond to unallowed vehicles
            for (size_t t = 0; t < types.size(); ++t) {
                const auto& customers = types[t].avail_customers;
                for (size_t i = 1; i < customers.size(); ++i) {
                    if (!customers[i]) {
                        add_zero_constraint(i - 1, t);
                    }
                }
            }
        }

        const auto& V = prob.vehicles;

        // calculate dynamic weights for objective function
        {
            // customers:
            size_t non_zero_volume_customers = 0, non_zero_weight_customers = 0;
            for (const auto& c : prob.customers) {
                if (volume(c.demand) != 0)
                    non_zero_volume_customers++;
                if (weight(c.demand) != 0)
                    non_zero_weight_customers++;
            }

            // vehicles:
            size_t non_zero_volume_vehicles = 0, non_zero_weight_vehicles = 0;
            for (const auto& v : V) {
                if (volume(v.capacity) != 0)
                    non_zero_volume_vehicles++;
                if (weight(v.capacity) != 0)
                    non_zero_weight_vehicles++;
            }

            // alpha'_v = non_zero volumes / all volumes
            // alpha'_w = non_zero weights / all weights
            // alpha_v = alpha'_v / (alpha'_v + alpha'_w)
            // alpha_w = alpha'_w / (alpha'_v + alpha'_w)
            double alpha_prime_v = 0.0, alpha_prime_w = 0.0;
            alpha_prime_v = 0.5 * (non_zero_volume_customers / n_customers +
                                   non_zero_volume_vehicles / n_vehicles);
            alpha_prime_w = 0.5 * (non_zero_weight_customers / n_customers +
                                   non_zero_weight_vehicles / n_vehicles);
            double sum = alpha_prime_v + alpha_prime_w;
            m_alpha_v = alpha_prime_v / sum;
            m_alpha_w = alpha_prime_w / sum;
        }

        {
            // Note: double constraints due to volume && weight
            // (1)
            int total_volume = 0;
            int total_weight = 0;
            for (const auto& type : types) {
                const auto& vehicles = type.vehicles;
                total_volume += total(volume, V, vehicles);
                total_weight += total(weight, V, vehicles);
            }

            IloExprArray capacity_fractions(m_env);
            for (size_t t = 0; t < types.size(); ++t) {
                // total capacity for current model degrades into capacity of a
                // vehicle

                // volume case:
                IloExpr volume_fraction(m_env);
                if (total_volume != 0) {
                    for (size_t i = 1; i < n_customers; ++i) {
                        for (const auto& k : types[t].vehicles) {
                            const auto v = volume(V[k].capacity);
                            if (v == 0)
                                continue;
                            volume_fraction += m_x[i - 1][t] * v;
                        }
                    }
                }

                // weight case:
                IloExpr weight_fraction(m_env);
                if (total_weight != 0) {
                    for (size_t i = 1; i < n_customers; ++i) {
                        for (const auto& k : types[t].vehicles) {
                            const auto w = weight(V[k].capacity);
                            if (w == 0)
                                continue;
                            weight_fraction += m_x[i - 1][t] * w;
                        }
                    }
                }

                // total = volume + weight
                if (total_volume == 0) {
                    capacity_fractions.add(m_alpha_w * weight_fraction /
                                           total_weight);
                } else if (total_weight == 0) {
                    capacity_fractions.add(m_alpha_v * volume_fraction /
                                           total_volume);
                } else {
                    capacity_fractions.add(
                        m_alpha_w * weight_fraction / total_weight +
                        m_alpha_v * volume_fraction / total_volume);
                }
            }
            m_capacity_fractions = capacity_fractions;
            m_objective = IloMinimize(m_env, IloMax(m_capacity_fractions));
            m_model.add(m_objective);
        }

        {
            // (2)
            IloConstraintArray allowability_constraints(m_env);
            for (size_t i = 1; i < n_customers; ++i) {
                const auto& array = m_x[i - 1];
                IloExpr sum(m_env);
                for (IloInt t = 0; t < array.getSize(); ++t) {
                    sum += array[t];
                }
                allowability_constraints.add(sum == 1);
            }
            m_model.add(allowability_constraints);
        }

        {
            // (3)
            IloConstraintArray balancing_constraints(m_env);
            for (size_t t = 0; t < types.size(); ++t) {
                const auto& vehicles = types[t].vehicles;

                // volume case:
                const auto total_volume = total(volume, V, vehicles);
                if (total_volume != 0) {
                    IloExpr volume_sum(m_env);
                    for (size_t i = 1; i < n_customers; ++i) {
                        const auto v = volume(prob.customers[i].demand);
                        if (v == 0)
                            continue;
                        // TODO: questionable
                        volume_sum += v * m_x[i - 1][t];
                    }
                    balancing_constraints.add(volume_sum <=
                                              total_volume * m_objective);
                }

                // weight case:
                const auto total_weight = total(weight, V, vehicles);
                if (total_weight) {
                    IloExpr weight_sum(m_env);
                    for (size_t i = 1; i < n_customers; ++i) {
                        const auto w = weight(prob.customers[i].demand);
                        if (w == 0)
                            continue;
                        // TODO: questionable
                        weight_sum += w * m_x[i - 1][t];
                    }
                    balancing_constraints.add(weight_sum <=
                                              total_weight * m_objective);
                }
            }
            m_model.add(balancing_constraints);
        }

        {
            if (!prob.enable_splits()) {
                return;
            }

            // split delivery constraints
            IloConstraintArray limit_constraints1(m_env),
                limit_constraints2(m_env);

            assert(m_x.size() == m_y.size());
            for (size_t i = 0; i < m_x.size(); ++i) {
                assert(m_x[i].getSize() == m_y[i].getSize());
                for (int t = 0; t < m_x[i].getSize(); ++t) {
                    limit_constraints1.add(m_y[i][t] >= m_x[i][t]);
                    limit_constraints2.add(m_y[i][t] <=
                                           m_x[i][t] + (1.0 - SPLIT_THR));
                }
            }
            m_model.add(limit_constraints1);
            m_model.add(limit_constraints2);

            const int split_value = prob.max_splits;
            IloConstraintArray split_constraints(m_env);
            for (size_t i = 0; i < m_y.size(); ++i) {
                IloExpr sum(m_env);
                for (int t = 0; t < m_y[i].getSize(); ++t) {
                    sum += m_y[i][t];
                }
                split_constraints.add(sum <= split_value);
            }
            m_model.add(split_constraints);
        }
    }

    ~Heuristic() { m_algo.end(); }

    void solve() {
        if (!m_algo.solve()) {
            static const constexpr char msg[] = "IloAlgorithm::solve() failed!";
            LOG_ERROR << msg << "\n";
            throw std::runtime_error(msg);
        }
    }

    std::vector<double> get_values(size_t i) const {
        const auto& vars = m_x[i];
        IloNumArray vals(m_env);
        m_algo.getValues(vals, vars);
        std::vector<double> converted{};
        converted.reserve(vals.getSize());
        for (IloInt i = 0; i < vals.getSize(); ++i) {
            converted.emplace_back(static_cast<double>(vals[i]));
        }
        return converted;
    }

    std::vector<std::vector<double>> get_values() const {
        std::vector<std::vector<double>> vals{};
        vals.reserve(m_x.size());
        for (size_t i = 0; i < m_x.size(); ++i) {
            vals.emplace_back(std::move(get_values(i)));
        }
        return vals;
    }

    /// Get mapping between customer and vehicle type for current solution
    std::unordered_map<size_t, size_t> get_customer_types(size_t depot_offset) {
        auto assignment_map = this->get_values();
        std::unordered_map<size_t, size_t> mapped_types;
        for (size_t c = 0; c < assignment_map.size(); ++c) {
            const auto& ratios = assignment_map[c];
            // TODO: fix this for split case - need to update assignment_cost as
            //       well then
            auto chosen_type = std::max_element(ratios.cbegin(), ratios.cend());
            size_t t = std::distance(ratios.cbegin(), chosen_type);
            mapped_types[c + depot_offset] = t;
        }
        return mapped_types;
    }

    void update() {
        auto seeds = select_seeds(*this);
        const auto n_customers = m_prob.n_customers();
        auto customer_to_type = this->get_customer_types(1);

        // (9) calculate total minimal insertion cost
        double total_distance = 0.0;
        for (size_t k = 1; k < n_customers; ++k) {
            const auto& allowed = m_prob.allowed_types(k);
            for (size_t t = 0; t < allowed.size(); ++t) {
                if (allowed[t]) {
                    total_distance +=
                        assignment_cost(seeds.at(customer_to_type[k]), k);
                }
            }
        }

        // (9) calculate normalized minimal insertion cost as a CPLEX function
        IloExpr insertion_costs(m_env);
        for (size_t i = 1; i < n_customers; ++i) {
            const auto& array = m_x[i - 1];

            // this must hold true because we operate on types:
            assert(static_cast<size_t>(array.getSize()) == seeds.size());

            for (IloInt t = 0; t < array.getSize(); ++t) {
                insertion_costs += assignment_cost(seeds.at(t), i) * array[t];
            }
        }
        m_model.remove(m_objective);
        m_objective =
            IloMinimize(m_env, IloExpr(insertion_costs / total_distance +
                                       IloMax(m_capacity_fractions)));
        m_model.add(m_objective);
    }
};

inline double divide(int divident, int divider) {
    if (divider == 0) {
        return 0.0;
    }
    return static_cast<double>(divident) / divider;
}

/// (6) Calculate weight of each customer
std::vector<double> calculate_weights(const Problem& prob) {
    const auto size = prob.n_customers();
    const auto& depot_costs = prob.costs[0];
    const auto max_cost =
        *std::max_element(depot_costs.cbegin(), depot_costs.cend());
    const auto max = std::max_element(
        prob.customers.cbegin() + 1, prob.customers.cend(),
        [](const auto& a, const auto& b) { return a.demand < b.demand; });
    const auto max_volume = volume(max->demand);
    const auto max_weight = weight(max->demand);
    std::vector<double> weights(size, 0.0);
    for (size_t i = 1; i < size; ++i) {
        weights[i] = (divide(volume(prob.customers[i].demand), max_volume) +
                      divide(weight(prob.customers[i].demand), max_weight)) +
                     (depot_costs[i] / max_cost);
    }
    return weights;
}

/// Fix type ratios (split delivery)
std::vector<double> fix_ratios(const TransportationQuantity& demand,
                               const std::vector<double>& ratios) {
    // TODO: simplify algorithm

    // skip non-floating cases - [0, ..., 1.0, ..., 0]
    if (ratios.cend() != std::find(ratios.cbegin(), ratios.cend(), 1.0)) {
        return ratios;
    }

    std::unordered_map<size_t, double> ratio_map;
    ratio_map.reserve(ratios.size());
    // insert elements > 0.0
    for (size_t t = 0, size = ratios.size(); t < size; ++t) {
        auto r = ratios[t];
        if (r > 0.0) {
            ratio_map[t] = r;
        }
    }

    assert(demand.volume != 0);
    assert(demand.volume == demand.weight);
    const auto volume = demand.volume;

    // construct ratios aligned to demand
    const int grain_size = static_cast<int>(std::ceil(volume * SPLIT_THR));
    std::vector<double> aligned_ratios;
    for (int start = grain_size; start <= volume; ++start) {
        aligned_ratios.emplace_back(static_cast<double>(start) / volume);
    }

    // remember min ratio: "sacrifice" it in a way to properly align numbers
    auto min = std::min_element(
        ratio_map.begin(), ratio_map.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; });

    // find closest value to `e` in the given vector
    static const auto closest = [](std::vector<double>& values,
                                   double e) -> double {
        assert(!values.empty());
        auto greater_or_equal =
            std::lower_bound(values.cbegin(), values.cend(), e);
        if (greater_or_equal == values.cend()) {
            return *std::prev(values.cend());
        }
        if (greater_or_equal == values.cbegin()) {
            return *greater_or_equal;
        }

        auto lower = std::prev(greater_or_equal);
        // if e is closer to lower, return lower. otherwise, return lower_bound
        if (e - *lower < *greater_or_equal - e) {
            return *lower;
        }
        return *greater_or_equal;
    };

    // align ratios
    double sum = 0.0;
    for (auto& p : ratio_map) {
        if (p.first == min->first) {
            continue;
        }
        p.second = closest(aligned_ratios, p.second);
        sum += p.second;
    }
    min->second = 1.0 - sum;  // min value is guaranteed to be aligned, because
                              // other values are already aligned

    {
        // if one of the elements becomes 1.0, fix all other values to be 0.0
        auto found_one =
            std::find_if(ratio_map.cbegin(), ratio_map.cend(),
                         [](const auto& p) { return p.second == 1.0; });
        if (found_one != ratio_map.cend()) {
            for (auto& p : ratio_map) {
                if (p.first == found_one->first) {
                    continue;
                }
                p.second = 0.0;
            }
        }
    }

    // construct result
    std::vector<double> fixed_ratios(ratios.size(), 0.0);
    for (const auto& p : ratio_map) {
        fixed_ratios[p.first] = p.second;
    }

    assert(1.0 ==
           std::accumulate(fixed_ratios.cbegin(), fixed_ratios.cend(), 0.0));

    return fixed_ratios;
}

/// Get non-constructed groups of customers that belong to the same routes
std::pair<std::unordered_map<size_t, std::list<size_t>>,
          std::unordered_map<size_t, SplitInfo>>
group(const Heuristic& h, size_t depot_offset) {
    auto assignment_map = h.get_values();
    std::unordered_map<size_t, std::list<size_t>> routes;
    std::unordered_map<size_t, SplitInfo> splits;
    for (size_t c = 0; c < assignment_map.size(); ++c) {
        auto ratios = fix_ratios(h.prob().customers[c + depot_offset].demand,
                                 assignment_map[c]);
        for (size_t t = 0; t < ratios.size(); ++t) {
            if (ratios[t] == 0.0) {
                continue;
            }
            routes[t].emplace_back(c + depot_offset);
            splits[c + depot_offset].split_info[t] = ratios[t];
        }
    }
    return std::make_pair(routes, splits);
}

template<typename T> using mat_t = std::vector<std::vector<T>>;

template<typename T, typename ListIt>
T sum_route_part(const mat_t<T>& mat, ListIt first, ListIt last) {
    T sum = static_cast<T>(0);
    for (auto first2 = std::next(first); first2 != last; ++first, ++first2) {
        sum += mat[*first][*first2];
    }
    return sum;
}

#define EXPERIMENTAL 1

constexpr const double VRP_RANDOMNESS_THRESHOLD = 0.8;

std::tuple<TransportationQuantity, double, double>
get_statistics(const Problem& prob) {
    const auto& vehicles = prob.vehicles;
    assert(!vehicles.empty());
    TransportationQuantity max_capacity =
        std::max_element(vehicles.cbegin(), vehicles.cend(),
                         [](const Vehicle& a, const Vehicle& b) {
                             return a.capacity < b.capacity;
                         })
            ->capacity;

    double max_fixed_cost =
        std::max_element(vehicles.cbegin(), vehicles.cend(),
                         [](const Vehicle& a, const Vehicle& b) {
                             return a.fixed_cost < b.fixed_cost;
                         })
            ->fixed_cost;

    double max_variable_cost =
        std::max_element(vehicles.cbegin(), vehicles.cend(),
                         [](const Vehicle& a, const Vehicle& b) {
                             return a.variable_cost < b.variable_cost;
                         })
            ->variable_cost;

    if (max_fixed_cost == 0.0) {
        max_fixed_cost = 1.0;
    }
    if (max_variable_cost == 0.0) {
        max_variable_cost = 1.0;
    }

    return std::make_tuple(max_capacity, max_fixed_cost, max_variable_cost);
}

/// Solve basic (capacitated) VRP with insertion heuristic
std::pair<std::vector<std::tuple<size_t, size_t, std::list<size_t>>>,
          std::unordered_map<size_t, SplitInfo>>
solve_vrp(const Heuristic& h, bool random = false) {
    std::unordered_map<size_t, std::list<size_t>> typed_customers;
    std::unordered_map<size_t, SplitInfo> customer_splits;
    // depot_offset = 1 due to depot at index 0:
    std::tie(typed_customers, customer_splits) = group(h, 1);

    const auto& prob = h.prob();
    static constexpr const size_t depot = 0;
    std::unordered_map<size_t, std::list<size_t>> routes{};
    std::unordered_map<size_t, size_t> vehicle_to_type{};
    // perform allocations:
    const auto n_vehicles = prob.n_vehicles();
    routes.reserve(n_vehicles);  // allocate for max number of vehicles
    vehicle_to_type.reserve(n_vehicles);
    const auto& all_types = prob.vehicle_types();
    for (const auto& pair : typed_customers) {
        for (size_t v : all_types[pair.first].vehicles) {
            routes[v] = {};
            vehicle_to_type[v] = pair.first;
        }
    }

    TransportationQuantity max_cap = {};
    double max_fixed = 0.0, max_variable = 0.0;
    std::tie(max_cap, max_fixed, max_variable) = get_statistics(h.prob());
    const auto& prob_vehicles = h.prob().vehicles;
    const auto vehicle_value = [max_cap, max_fixed, max_variable,
                                &prob_vehicles](size_t i) {
        const auto& v = prob_vehicles[i];
        const auto fraction = v.capacity / max_cap;
        return (v.fixed_cost / max_fixed + v.variable_cost / max_variable) -
               (fraction.volume + fraction.weight);
    };

    static std::mt19937 g;
    static std::uniform_real_distribution<> dist(0.0, 1.0);

    std::unordered_map<size_t, SplitInfo> splits_by_vehicles;

    // Insertion heuristic, variation 2: c1 is not needed (?), c2 is minimized.
    // params of c2:
    static constexpr const double beta_1 = 0.5, beta_2 = 0.5;
    for (auto& type_and_customers : typed_customers) {
        auto t = type_and_customers.first;
        auto unrouted = type_and_customers.second;
        unrouted.sort([&prob](auto a, auto b) {
            return prob.customers[a].hard_tw.first <
                   prob.customers[b].hard_tw.first;
        });

        // sort vehicles by specific value function
        auto vehicles = all_types[t].vehicles;
        std::sort(vehicles.begin(), vehicles.end(),
                  [&prob_vehicles, &vehicle_value](size_t a, size_t b) {
                      return vehicle_value(a) < vehicle_value(b);
                  });

        bool last_vehicle = false;
        for (size_t i = 0; i < vehicles.size(); ++i) {
            // FIXME: push everything in the last vehicle - no choice (?)
            if (i == vehicles.size() - 1)
                last_vehicle = true;
            const size_t v = vehicles[i];
            TransportationQuantity running_capacity = prob.vehicles[v].capacity;
            // init
            auto& route = routes[v];
            route.emplace_front(depot);
            route.emplace_back(depot);
            bool nothing_to_add = false;
            while (!nothing_to_add) {
                nothing_to_add = true;
                // format: customer_id, c2 value, best insertion position
                using opt_data_t = std::tuple<size_t, double, size_t>;
                std::list<opt_data_t> optimal_c2{};
                for (const auto& c : unrouted) {
                    // skip if capacity is exceeded
                    if (!last_vehicle &&
                        running_capacity <
                            prob.customers[c].demand *
                                customer_splits[c].split_info[t]) {
                        continue;
                    }
                    if (!last_vehicle && random &&
                        dist(g) > VRP_RANDOMNESS_THRESHOLD) {
                        continue;
                    }
                    std::list<double> ratings_c2{};
                    for (auto i = route.cbegin(), j = std::next(i);
                         j != route.cend(); ++i, ++j) {
                        // calculate total route distance with `c` included:
                        // 0->i + (i->c + c->j) + j->0
                        auto route_dist =
                            prob.costs[*i][c] + prob.costs[c][*j] +
                            sum_route_part(prob.costs, route.cbegin(), j) +
                            sum_route_part(prob.costs, j, route.cend());
                        // calculate total route time with `c` included:
                        // 0->i + (i->c + c->j) + j->0
                        auto route_time =
                            prob.times[*i][c] + prob.times[c][*j] +
                            sum_route_part(prob.times, route.cbegin(), j) +
                            sum_route_part(prob.times, j, route.cend());
                        // total distance + total time:
                        ratings_c2.emplace_back(beta_1 * route_dist +
                                                beta_2 * route_time);
// TODO: does this even work?
#if EXPERIMENTAL  // TODO: verify it's worth it. may work better without it!
                  // check optimal is good enough to be included:
                        if (!last_vehicle) {
                            auto cost = prob.costs[0][c];
                            // TODO: add assignment_time as well?
                            if (assignment_cost(prob, *i, c) > cost ||
                                assignment_cost(prob, *j, c) > cost) {
                                continue;
                            }
                        }
#endif
                    }
                    if (ratings_c2.empty())
                        continue;
                    auto min = std::min_element(ratings_c2.cbegin(),
                                                ratings_c2.cend());
                    // Note: (distance + 1) to relate to actual route's start at
                    // depot. otherwise, we'd insert before depot
                    optimal_c2.emplace_back(std::make_tuple(
                        c, *min, std::distance(ratings_c2.cbegin(), min) + 1));
                }

                // might occur due to customers skip (e.g. capacity < demand)
                if (optimal_c2.empty())
                    continue;

                // find optimal customer and update route
                optimal_c2.sort([](const opt_data_t& a, const opt_data_t& b) {
                    return std::get<1>(a) < std::get<1>(b);
                });
                auto optimal = optimal_c2.cbegin();

                // handle time window constraints:
                std::decay_t<decltype(route)> updated_route = route;
                if (!last_vehicle) {
                    for (; optimal != optimal_c2.cend(); ++optimal) {
                        updated_route.insert(std::next(updated_route.cbegin(),
                                                       std::get<2>(*optimal)),
                                             std::get<0>(*optimal));
                        SplitInfo info = {};
                        for (size_t c : updated_route) {
                            info.split_info[c] =
                                customer_splits[c].split_info[t];
                        }
                        if (constraints::total_violated_time(
                                prob, info, updated_route.cbegin(),
                                updated_route.cend()) == 0) {
                            break;
                        }
                        updated_route = route;
                    }

                    if (optimal == optimal_c2.cend()) {
                        continue;
                    }
                } else {
                    updated_route.insert(std::next(updated_route.cbegin(),
                                                   std::get<2>(*optimal)),
                                         std::get<0>(*optimal));
                }

                const auto& c = std::get<0>(*optimal);
                unrouted.remove(c);
                route = std::move(updated_route);
                assert(route.size() > 2);

                running_capacity -=
                    prob.customers[c].demand * customer_splits[c].split_info[t];
                nothing_to_add = false;

                // convert types to vehicles in SplitInfo
                splits_by_vehicles[c].split_info[v] =
                    customer_splits[c].split_info[t];
            }
        }
    }

    assert(splits_by_vehicles.size() == customer_splits.size() - 1);

    // return only real routes (if route consists of <= 2 nodes, it's actually
    // empty - "size 2" stands for in-depot and out-depot)
    std::vector<std::tuple<size_t, size_t, std::list<size_t>>> cleaned_routes{};
    cleaned_routes.reserve(routes.size());
    for (auto& vehicle_and_route : routes) {
        if (vehicle_and_route.second.size() > 2) {
            const auto v = vehicle_and_route.first;
            cleaned_routes.emplace_back(vehicle_to_type[v], v,
                                        std::move(vehicle_and_route.second));
        }
    }

    return std::make_pair(cleaned_routes, splits_by_vehicles);
}

Solution routes_to_sln(
    const Problem& prob,
    std::pair<std::vector<std::tuple<size_t, size_t, std::list<size_t>>>,
              std::unordered_map<size_t, SplitInfo>>
        routes_and_splits) {
    const auto& routes = routes_and_splits.first;
    const auto& split_by_vehicles = routes_and_splits.second;

    Solution sln;
    sln.routes.reserve(routes.size());

    std::unordered_map<size_t, SplitInfo> splits_by_routes;
    for (auto& values : routes) {
        const size_t vehicle = std::get<1>(values);
        sln.routes.emplace_back(vehicle, std::move(std::get<2>(values)));

        // convert vehicles to routes in SplitInfo
        const size_t rid = sln.routes.size() - 1;  // always last inserted route
        for (const auto& p : split_by_vehicles) {
            if (!p.second.has(vehicle)) {
                continue;
            }
            splits_by_routes[p.first].split_info[rid] =
                p.second.split_info.at(vehicle);
        }
    }

    // if split delivery is disabled, do not set split info
    if (!prob.enable_splits()) {
        return sln;
    }

    assert(splits_by_routes.size() == split_by_vehicles.size());

    // transform <customer: SplitInfo{route, ratio}> into
    // <route: SplitInfo{customer, ratio}>
    SplitInfo depot_info = {};
    depot_info.split_info[0] = 1.0;
    sln.route_splits.resize(sln.routes.size(), depot_info);
    for (const auto& customer_and_splits : splits_by_routes) {
        auto c = customer_and_splits.first;
        const auto& info = customer_and_splits.second;
        for (const auto& e : info.split_info) {
            sln.route_splits[e.first].split_info[c] = e.second;
        }
    }

    return sln;
}

/// Select seeds for each route
std::unordered_map<size_t, std::vector<size_t>>
select_seeds(const Heuristic& h) {
    const auto& prob = h.prob();
    auto weights = calculate_weights(prob);

    // the customer on the route with the largest seed weight becomes the seed
    // point of the route
    auto routes = solve_vrp(h).first;  // TODO: can ignore splits here?
    // TODO: in fact, we don't need routes...
    for (auto& vehicle_route : routes) {
        auto& route = std::get<2>(vehicle_route);
        route.sort(
            [&weights](size_t i, size_t j) { return weights[i] > weights[j]; });
    }
    // sort all routes to put bigger in the beginning
    std::sort(routes.begin(), routes.end(), [](const auto& a, const auto& b) {
        return std::get<2>(a).size() > std::get<2>(b).size();
    });

    const auto size = prob.n_vehicles();
    unordered_map<size_t, std::vector<size_t>> seeds{};

    std::unordered_map<size_t, uint32_t> number_of_seeds_per_route = {};
    number_of_seeds_per_route.reserve(size);
    for (size_t v = 0; v < size; ++v) {
        size_t route_index = v % routes.size();
        const auto* route = &std::get<2>(routes[route_index]);
        size_t node_index = number_of_seeds_per_route[route_index];
        // skip "exhausted" routes:
        size_t v_next = v;
        while (route->size() <= node_index + 2) {
            v_next++;
            route_index = v_next % routes.size();
            route = &std::get<2>(routes[route_index]);
            node_index = number_of_seeds_per_route[route_index];
        }
        const auto type = std::get<0>(routes[route_index]);
        seeds[type].reserve(size);
        seeds[type].emplace_back(*std::next(route->cbegin(), node_index));
        number_of_seeds_per_route[route_index]++;
    }

    // this must hold due to algorithm: seeds size == number of vehicles
    assert(std::accumulate(seeds.cbegin(), seeds.cend(), size_t(0),
                           [](size_t sum, const auto& s) -> size_t {
                               return sum + s.second.size();
                           }) == size);

    return seeds;
}

std::vector<Solution> construct_solutions(const Heuristic& h, size_t count) {
    const bool solve_randomly = count > 1;
    std::vector<Solution> solutions(count);
    for (size_t i = 0, size = solutions.size(); i < size; ++i) {
        solutions[i] = std::move(
            routes_to_sln(h.prob(), solve_vrp(h, i && solve_randomly)));
    }
    return solutions;
}
}  // namespace

std::vector<Solution> cfrs_impl(const Problem& prob, size_t count) {
    Heuristic h(prob);
    h.solve();
#ifndef NDEBUG
    LOG_INFO << "(CPLEX) Objective = " << h.algo().getObjValue() << EOL;
#endif
    // loop here:
    h.update();
    h.solve();
#ifndef NDEBUG
    LOG_INFO << "(CPLEX) Objective = " << h.algo().getObjValue() << EOL;
#endif

    return construct_solutions(h, count);
}

}  // namespace detail
}  // namespace vrp
