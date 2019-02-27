#include "cluster_first_route_second.h"

#include <cassert>
#include <algorithm>
#include <random>
#include <numeric>
#include <utility>
#include <iostream>
#include <stdexcept>
#include <list>
#include <unordered_map>

#define ILOUSESTL
using namespace std;
#include "ilcplex/ilocplex.h"

/// Specify that the variable x is unused in the code
#define UNUSED(x) (void)x

/// INFO stream
#define LOG_INFO std::cout << "[INFO] "

/// ERROR stream
#define LOG_ERROR std::cerr << "[ERROR] "

/// End of line, without stream flushing
#define EOL "\n"

/// End of line, with stream flushing
#define EOL_FLUSH std::endl

namespace vrp {
namespace detail {

namespace {
inline int volume(const TransportationQuantity& q) {
    return q.volume;
}
inline int weight(const TransportationQuantity& q) {
    return q.weight;
}

template<typename AttribAccessor>
int total(AttribAccessor accessor, const std::vector<Vehicle>& vehicles,
    const std::vector<size_t>& indices) {
    return std::accumulate(indices.cbegin(), indices.cend(), 0,
        [&] (int val, size_t i) {
            return val + accessor(vehicles[i].capacity); });
}

/// Cost of assigning customer i to vehicle t. Single vehicle in type case.
inline double assignment_cost(const Problem& prob, size_t seed, size_t i) {
    const auto& c = prob.costs;
    return c[0][i] + c[i][seed] - c[0][seed];
}

/// Cost of assigning customer i to vehicle t. General case.
double assignment_cost(const Problem& prob, const std::vector<size_t>& seeds,
    size_t i) {
    const auto size = seeds.size();
    std::vector<double> costs(size);
    for (size_t s = 0; s < size; ++s) {
        costs[s] = assignment_cost(prob, seeds[s], i);
    }
    return *std::min_element(costs.cbegin(), costs.cend());
}

/// Heuristic class that solves the relaxed 0-1 Integer Problem
class Heuristic {
    const Problem& m_prob;

    IloEnv m_env;
    std::vector<IloIntVarArray> m_x;  ///< x[i][t]: 1 if customer i is assigned
                                      /// to vehicle t. index is customer
    IloModel m_model = IloModel(m_env);
    IloCplex m_algo = IloCplex(m_model);
    IloObjective m_objective;
    IloExprArray m_capacity_fractions;

    inline void add_zero_constraint(size_t i, IloInt t) {
        IloExpr zero_expr(m_env);
        zero_expr = m_x[i][t];
        m_model.add(IloConstraint(zero_expr == 0));
    }

    /// Cost of assigning customer i to vehicle t. Single vehicle in type case.
    inline double assignment_cost(size_t seed, size_t i) {
        return ::vrp::detail::assignment_cost(m_prob, seed, i);
    }

    /// Cost of assigning customer i to vehicle t. General case.
    inline double assignment_cost(const std::vector<size_t>& seeds, size_t i) {
        return ::vrp::detail::assignment_cost(m_prob, seeds, i);
    }


public:
    inline IloEnv& env() { return m_env; }
    inline std::vector<IloIntVarArray>& X() { return m_x; }
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
        const auto& types = prob.vehicle_types();

        for (size_t i = 1; i < n_customers; ++i) {
            const auto size = types.size();
            m_x.emplace_back(IloIntVarArray(m_env, size, 0.0, 1.0));
        }

        {
            // TODO: too many constraints??

            // zero-out variables that correspond to unallowed vehicles
            for (size_t t = 0; t < types.size(); ++t) {
                const auto& customers = types[t].avail_customers;
                for (size_t i = 1; i < customers.size(); ++i) {
                    if (!customers[i]) {
                        add_zero_constraint(i-1, t);
                    }
                }
            }
        }

        const auto& V = prob.vehicles;
        {
            // Note: double constraints due to volume && weight
            // (1)
            // TODO: is the function correct?
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
                            if (v == 0) continue;
                            volume_fraction += m_x[i-1][t] * v;
                        }
                    }
                }

                // weight case:
                IloExpr weight_fraction(m_env);
                if (total_weight != 0) {
                    for (size_t i = 1; i < n_customers; ++i) {
                        for (const auto& k : types[t].vehicles) {
                            const auto w = weight(V[k].capacity);
                            if (w == 0) continue;
                            weight_fraction += m_x[i-1][t] * w;
                        }
                    }
                }

                // total = volume + weight
                if (total_volume == 0) {
                    capacity_fractions.add(weight_fraction / total_weight);
                } else if (total_weight == 0) {
                    capacity_fractions.add(volume_fraction / total_volume);
                } else {
                    capacity_fractions.add(weight_fraction / total_weight
                        + volume_fraction / total_volume);
                }
            }
            m_capacity_fractions = capacity_fractions;
            m_objective = IloMinimize(m_env, IloMax(m_capacity_fractions));
            m_model.add(m_objective);
        }

        {
            // (2)
            IloConstraintArray allowability_constraints(m_env);
            for (size_t i = 1; i < n_customers; ++i){
                const auto& array = m_x[i-1];
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
            // TODO: is this correct?
            IloConstraintArray balancing_constraints(m_env);
            for (size_t t = 0; t < types.size(); ++t) {
                const auto& vehicles = types[t].vehicles;

                // volume case:
                const auto total_volume = total(volume, V, vehicles);
                if (total_volume != 0) {
                    IloExpr volume_sum(m_env);
                    for (size_t i = 1; i < n_customers; ++i) {
                        const auto v = volume(prob.customers[i].demand);
                        if (v == 0) continue;
                        // TODO: questionable
                        volume_sum += v * m_x[i-1][t];
                    }
                    balancing_constraints.add(
                        volume_sum <= total_volume * m_objective);
                }

                // weight case:
                const auto total_weight = total(weight, V, vehicles);
                if (total_weight) {
                    IloExpr weight_sum(m_env);
                    for (size_t i = 1; i < n_customers; ++i) {
                        const auto w = weight(prob.customers[i].demand);
                        if (w == 0) continue;
                        // TODO: questionable
                        weight_sum += w * m_x[i-1][t];
                    }
                    balancing_constraints.add(
                        weight_sum <= total_weight * m_objective);
                }
            }
            m_model.add(balancing_constraints);
        }
    }

    ~Heuristic() {
        m_algo.end();
    }

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

    void update_with_seeds(const std::vector<size_t>& seeds) {
        const auto n_customers = m_prob.n_customers();
        // (9) calculate total minimal insertion cost
        double total_distance = 0.0;
        for (size_t k = 1; k < n_customers; ++k) {
            const auto& allowed = m_prob.allowed_vehicles(k);
            // TODO: fix this
            assert(allowed.size() == seeds.size());
            for (size_t t = 0; t < allowed.size(); ++t) {
                if (allowed[t]) {
                    total_distance += assignment_cost(seeds[t], k);
                }
            }
        }

        // (9) calculate normalized minimal insertion cost as a CPLEX function
        IloExpr insertion_costs(m_env);
        for (size_t i = 1; i < n_customers; ++i) {
            const auto& array = m_x[i-1];
            assert(static_cast<size_t>(array.getSize()) == seeds.size());
            for (IloInt t = 0; t < array.getSize(); ++t) {
                insertion_costs += assignment_cost(seeds[t], i) * array[t];
            }
        }
        m_model.remove(m_objective);
        m_objective = IloMinimize(m_env,
            IloExpr(insertion_costs / total_distance
                    + IloMax(m_capacity_fractions)));
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
    const auto max_cost = *std::max_element(depot_costs.cbegin(),
        depot_costs.cend());
    const auto max = std::max_element(prob.customers.cbegin()+1,
        prob.customers.cend(), [] (const auto& a, const auto& b) {
            return a.demand < b.demand; });
    const auto max_volume = volume(max->demand);
    const auto max_weight = weight(max->demand);
    std::vector<double> weights(size - 1, 0);
    for (size_t i = 1; i < size; ++i) {
        weights[i-1] =
            (divide(volume(prob.customers[i].demand), max_volume)
            + divide(weight(prob.customers[i].demand), max_weight))
            + (depot_costs[i] / max_cost);
    }
    return weights;
}

/// Get non-constructed groups of customers that belong to the same routes
std::unordered_map<size_t, std::list<size_t>> group(const Heuristic& h,
    size_t depot_offset = 0) {
    auto assignment_map = h.get_values();
    std::unordered_map<size_t, std::list<size_t>> routes;
    for (size_t c = 0; c < assignment_map.size(); ++c) {
        const auto& types_for_customer = assignment_map[c];
        auto chosen_vehicle_type = std::find(types_for_customer.cbegin(),
            types_for_customer.cend(), 1.0);
        size_t chosen_vehicle_type_index = std::distance(
            types_for_customer.cbegin(), chosen_vehicle_type);
        routes[chosen_vehicle_type_index].emplace_back(c + depot_offset);
    }
    return routes;
}

template<typename T>
using mat_t = std::vector<std::vector<T>>;

template<typename T, typename ListIt>
T sum_route_part(const mat_t<T>& mat, ListIt first, ListIt last) {
    T sum = static_cast<T>(0);
    for (auto first2 = std::next(first, 1); first2 != last; ++first, ++first2) {
        sum += mat[*first][*first2];
    }
    return sum;
}

#define EXPERIMENTAL 1

/// Solve basic (capacitated) VRP (CVRP) with insertion heuristic
std::vector<std::pair<size_t, std::list<size_t>>>
solve_cvrp(const Heuristic& h) {
    // depot_offset = 1 due to depot at index 0:
    auto typed_customers = group(h, 1);
    const auto& prob = h.prob();
    static constexpr const size_t depot = 0;
    // std::unordered_map<size_t, size_t> vehicle_to_type = {};
    // vehicle_to_type.reserve(prob.n_vehicles());
    std::unordered_map<size_t, std::list<size_t>> routes = {};
    // perform allocations:
    routes.reserve(prob.n_vehicles());  // allocate for max number of vehicles
    const auto& all_types = prob.vehicle_types();
    for (const auto& pair : typed_customers) {
        for (size_t v : all_types[pair.first].vehicles) {
            // vehicle_to_type[v] = pair.first;
            routes[v] = {};
        }
    }

    // Insertion heuristic, variation 2: c1 is not needed (?), c2 is minimized.
    // params of c2:
    static constexpr const double beta_1 = 0.5, beta_2 = 0.5;
    for (auto& type_and_customers : typed_customers) {
        auto unrouted = type_and_customers.second;
        // TODO: sort vehicles by some criterion (e.g. capacity/cost efficiency)
        const auto& vehicles = all_types[type_and_customers.first].vehicles;
        bool last_vehicle = false;
        for (size_t i = 0; i < vehicles.size(); ++i) {
            // FIXME: push everything in the last vehicle - no choice (?)
            if (i == vehicles.size() - 1) last_vehicle = true;
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
                    if (!last_vehicle
                        && running_capacity < prob.customers[c].demand)
                            continue;
                    std::list<double> ratings_c2{};
                    for (auto i = route.cbegin(), j = std::next(i, 1);
                        j != route.cend(); ++i, ++j) {
                        // calculate total route distance with `c` included:
                        // 0->i + (i->c + c->j) + j->0
                        auto route_dist = prob.costs[*i][c] + prob.costs[c][*j]
                            + sum_route_part(prob.costs, route.cbegin(), j)
                            + sum_route_part(prob.costs, j, route.cend());
                        // calculate total route time with `c` included:
                        // 0->i + (i->c + c->j) + j->0
                        auto route_time = prob.times[*i][c] + prob.times[c][*j]
                            + sum_route_part(prob.times, route.cbegin(), j)
                            + sum_route_part(prob.times, j, route.cend());
                        // total distance + total time:
                        ratings_c2.emplace_back(
                            beta_1*route_dist + beta_2*route_time);
// TODO: does this even work?
#if EXPERIMENTAL  // TODO: verify it's worth it. may work better without it!
                        // check optimal is good enough to be included:
                        if (!last_vehicle) {
                            auto cost = prob.costs[0][c];
                            // TODO: add assignment_time as well?
                            if (assignment_cost(prob, *i, c) > cost
                                || assignment_cost(prob, *j, c) > cost) {
                                continue;
                            }
                        }
#endif
                    }
                    if (ratings_c2.empty()) continue;
                    auto min = std::min_element(ratings_c2.cbegin(),
                        ratings_c2.cend());
                    // Note: (distance + 1) to relate to actual route's start at
                    // depot. otherwise, we'd insert before depot
                    optimal_c2.emplace_back(std::make_tuple(
                        c, *min, std::distance(ratings_c2.cbegin(), min) + 1));
                }

                // might occur due to customers skip (e.g. capacity < demand)
                if (optimal_c2.empty()) continue;

                // find optimal customer and update route
                auto optimal = *std::min_element(
                    optimal_c2.cbegin(), optimal_c2.cend(),
                    [] (const opt_data_t& a, const opt_data_t& b) {
                        return std::get<1>(a) < std::get<1>(b);});
                const auto& c = std::get<0>(optimal);

                unrouted.remove(c);
                route.insert(
                    std::next(route.cbegin(), std::get<2>(optimal)), c);
                running_capacity -= prob.customers[c].demand;
                nothing_to_add = false;
            }
        }
    }

    // return only real routes (if route consists of <= 2 nodes, it's actually
    // empty - "size 2" stands for in-depot and out-depot)
    std::vector<std::pair<size_t, std::list<size_t>>> cleaned_routes = {};
    cleaned_routes.reserve(routes.size());
    for (auto& vehicle_and_route : routes) {
        if (vehicle_and_route.second.size() > 2) {
            cleaned_routes.emplace_back(std::move(vehicle_and_route));
        }
    }

    return cleaned_routes;
}

/// Select seeds for each route
std::vector<size_t> select_seeds(const Heuristic& h,
    const std::vector<double>& weights) {
    // TODO: handle case when seeds.size() != number of vehicles

    // the customer on the route with the largest seed weight becomes the seed
    // point of the route
    auto routes = solve_cvrp(h);
    std::vector<size_t> seeds{};
    seeds.reserve(routes.size());
    for (const auto& vehicle_route : routes) {
        const auto& route = vehicle_route.second;
        auto seed_customer = *std::max_element(route.cbegin(), route.cend(),
            [&weights] (size_t a, size_t b) {
                return weights[a] < weights[b]; });
        seeds.emplace_back(seed_customer + 1); // Note: +1 due to depot
    }
    return seeds;
}

std::vector<Solution> construct_solutions(const Heuristic& h, size_t count) {
    count = 1;  // TODO: add randomness
    std::vector<Solution> solutions(count);
#if 0
    // depot_offset = 1 due to depot at index 0. this is important here:
    auto routes = group(h, 1);
    // perform allocations:
    for (size_t i = 0; i < count; ++i) {
        solutions[i].routes.reserve(routes.size());
        for (const auto& pair : routes) {
            solutions[i].routes.emplace_back(std::make_pair(pair.first,
                std::list<size_t>{}));
        }
    }
    const auto& prob = h.prob();
    static constexpr const size_t depot = 0;
    // insertion heuristic, variation 2: c1 is not needed (?), c2 is minimized.
    // params of c2:
    static constexpr const double beta_1 = 0.5, beta_2 = 0.5;
#endif
    for (auto& sln : solutions) {
        sln.routes = std::move(solve_cvrp(h));
#if 0
        for (auto& vehicle_route : sln.routes) {
            // init
            auto& route = vehicle_route.second;
            route.emplace_front(depot);
            route.emplace_back(depot);
            auto unrouted = routes[vehicle_route.first];
            while (!unrouted.empty()) {
                // as route has a fixed cost/time outside of u "region", we can
                // simply calculate c2(i, u, j) as for route xx-i-u-j-xx without
                // adding xx nodes

                // format: customer_id, c2 value, best insertion position
                using opt_data_t = std::tuple<size_t, double, size_t>;
                std::list<opt_data_t> optimal_c2{};
                for (const auto& c : unrouted) {
                    std::list<double> ratings_c2{};
                    for (auto i = route.cbegin(), j = std::next(i, 1);
                        j != route.cend(); ++i, ++j) {
                        // calculate total route distance with `c` included:
                        // 0->i + (i->c + c->j) + j->0
                        auto route_dist = prob.costs[*i][c] + prob.costs[c][*j]
                            + sum_route_part(prob.costs, route.cbegin(), j)
                            + sum_route_part(prob.costs, j, route.cend());
                        // calculate total route time with `c` included:
                        // 0->i + (i->c + c->j) + j->0
                        auto route_time = prob.times[*i][c] + prob.times[c][*j]
                            + sum_route_part(prob.times, route.cbegin(), j)
                            + sum_route_part(prob.times, j, route.cend());
                        // total distance + total time:
                        ratings_c2.emplace_back(
                            beta_1*route_dist + beta_2*route_time);
                    }
                    auto min = std::min_element(ratings_c2.cbegin(),
                        ratings_c2.cend());
                    // Note: (distance + 1) to relate to actual route's start at
                    // depot. otherwise, we'd insert before depot
                    optimal_c2.emplace_back(std::make_tuple(
                        c, *min, std::distance(ratings_c2.cbegin(), min) + 1));
                }

                // find optimal customer and update route
                auto optimal = *std::min_element(
                    optimal_c2.cbegin(), optimal_c2.cend(),
                    [] (const opt_data_t& a, const opt_data_t& b) {
                        return std::get<1>(a) < std::get<1>(b);});
                const auto& c = std::get<0>(optimal);
                unrouted.remove(c);
                route.insert(
                    std::next(route.cbegin(), std::get<2>(optimal)), c);
            }
        }
#endif
    }
    return solutions;
}
}  // anonymous

std::vector<Solution> cfrs_impl(const Problem& prob, size_t count) {
    Heuristic h(prob);
    h.solve();
    LOG_INFO << "Objective = " << h.algo().getObjValue() << EOL;
    // we currently use vehicle_types_size == vehicles_size, so for each type
    // there's exactly one route. since we don't really care about actual route
    // construction right now, only about route seeds, we can skip route
    // construction and start updating heuristic right away
    auto weights = calculate_weights(prob);
    auto seeds = select_seeds(h, weights);
    h.update_with_seeds(seeds);
    h.solve();
    LOG_INFO << "Objective = " << h.algo().getObjValue() << EOL;
    return construct_solutions(h, count);
}

}  // detail
}  // vrp
