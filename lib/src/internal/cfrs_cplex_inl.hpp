#include "cluster_first_route_second.h"

#include <algorithm>
#include <cassert>
#include <random>
#include <numeric>
#include <utility>
#include <iostream>
#include <stdexcept>

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
template<typename T>
bool are_equal(T a, T b, int units_in_last_place = 2)
{
return std::abs(a - b) <= std::numeric_limits<T>::epsilon()
                            * std::max(std::abs(a), std::abs(b))
                            * units_in_last_place
        || std::abs(a - b) < std::numeric_limits<T>::min(); // subnormal result
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

    inline int index_of_vehicle(size_t customer, size_t vehicle) {
        const auto& vehicles = m_prob.customers[customer].suitable_vehicles;
        if (vehicles.empty()) {
            return -1;
        }
        auto v = std::find(vehicles.cbegin(), vehicles.cend(),
            static_cast<int>(vehicle));
        if (vehicles.cend() == v) {
            return -1;
        }
        return std::distance(vehicles.cbegin(), v);
    }

    void add_zero_constraint(size_t i, IloInt t) {
        IloExpr zero_expr(m_env);
        zero_expr = m_x[i][t];
        m_model.add(IloConstraint(zero_expr == 0));
    }

public:
    /// Reference: A Computational Study of a New Heuristic for the
    /// Site-Dependent Vehicle Routing Problem. Chao, Golden, Wasil. 1998
    Heuristic(const Problem& prob) : m_prob(prob) {
        // m_algo.setOut(m_env.getNullStream());

        const auto n_customers = prob.n_customers();

        for (size_t i = 1; i < n_customers; ++i) {
            const auto size = prob.allowed_vehicles(i).size();
            m_x.emplace_back(IloIntVarArray(m_env, size, 0.0, 1.0));
        }

        // TODO: too many constraints??
        // zero-out variables that correspond to unallowed vehicles
        for (size_t i = 1; i < n_customers; ++i) {
            const auto& allowed = prob.allowed_vehicles(i);
            for (IloInt t = 0; t < m_x[i-1].getSize(); ++t) {
                if (!allowed[t]) {
                    add_zero_constraint(i-1, t);
                }
            }
        }

        const auto& V = prob.vehicles;
        {
            // (1)
            // TODO: is the function correct?
            IloExprArray capacity_fractions(m_env);
            for (size_t t = 0; t < V.size(); ++t) {
                // total capacity for current model degrades into capacity of a
                // vehicle
                const auto total_capacity = V[t].capacity;
                IloExpr fraction(m_env);
                for (size_t i = 1; i < n_customers; ++i) {
                    // always 1 element in vehicle type for current model
                    fraction += m_x[i-1][t] * V[t].capacity;
                }
                capacity_fractions.add(fraction / total_capacity);
            }
            m_objective = IloMinimize(m_env, IloMax(capacity_fractions));
            m_model.add(m_objective);
        }

        {
            // (2)
            IloConstraintArray allowability_constraints(m_env);
            for (size_t i = 1; i < n_customers; ++i){
            // for (const auto& array : m_x) {
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
            for (size_t t = 0; t < V.size(); ++t) {
                int total_capacity = V[t].capacity;
                IloExpr sum(m_env);
                for (size_t i = 1; i < n_customers; ++i) {
                    const auto& c = prob.customers[i];
                    // TODO: the rest is questionable
                    // auto coeff = is_vehicle_allowed(t, i) * c.demand;
                    auto coeff = c.demand;
                    // auto index = index_of_vehicle(i, t);
                    // if (index < 0) continue;
                    sum += coeff * m_x[i-1][t];
                }
                balancing_constraints.add(sum <= total_capacity * m_objective);
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

    /// 1 if vehicle is allowable, 0 otherwise. a[t][i]
    int is_vehicle_allowed(size_t vehicle, size_t customer) const {
        const auto& vehicles = m_prob.customers[customer].suitable_vehicles;
        if (vehicles.empty()) {
            return 1;
        }
        bool allowed =  vehicles.cend() != std::find(vehicles.cbegin(),
            vehicles.cend(), static_cast<int>(vehicle));
        return static_cast<int>(allowed);
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

    inline IloEnv& env() { return m_env; }
    inline std::vector<IloIntVarArray>& X() { return m_x; }
    inline IloModel& model() { return m_model; }
    inline IloCplex& algo() { return m_algo; }
    inline IloObjective& objective() { return m_objective; }
};
}  // anonymous

std::vector<Solution> cfrs_impl(const Problem& prob, size_t count) {
    Heuristic h(prob);
    h.solve();
    auto obj_value = h.algo().getObjValue();
    LOG_INFO << "Objective = " << obj_value << EOL;
    auto all_values = h.get_values();
    UNUSED(obj_value);
    // TODO: find seeds. solve again with updated function
    return {};
}

}  // detail
}  // vrp
