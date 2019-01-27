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
    // TODO: solve Integer Problem instead of Non-Integer
    std::vector<IloNumVarArray> m_x;  ///< x[i][t]: 1 if customer i is assigned
                                      /// to vehicle t. index is customer
    IloModel m_model = IloModel(m_env);
    IloCplex m_algo = IloCplex(m_env);
    IloObjective m_objective;// = IloMinimize(m_env);
public:
    /// Reference: A Computational Study of a New Heuristic for the
    /// Site-Dependent Vehicle Routing Problem. Chao, Golden, Wasil. 1998
    Heuristic(const Problem& prob) : m_prob(prob) {
        // TODO: do I need to init these myself?
        for (size_t i = 1; i < prob.customers.size(); ++i) {
            auto size = prob.customers[i].suitable_vehicles.size();
            m_x.emplace_back(IloNumVarArray(m_env, size, 0.0, 1.0));
        }
        m_algo.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Concurrent);
        m_algo.setOut(m_env.getNullStream());

        // TODO: specify objective
        {
            IloExprArray capacity_fractions;
            const auto& V = prob.vehicles;
            auto total_capacity = std::accumulate(V.cbegin(), V.cend(), 0,
                [] (int init, const Vehicle& v) { return init + v.capacity; });
            for (size_t k = 0; k < V.size(); ++k) {
                IloExpr fraction(m_env);
                for (size_t i = 1; i < prob.customers.size(); ++i) {
                    for (IloInt t = 0; t < m_x[i-1].getSize(); ++t) {
                        fraction = m_x[i-1][t] * V[k].capacity / total_capacity;
                    }
                }
                capacity_fractions.add(fraction);
            }
            m_objective = IloMinimize(m_env, IloMax(capacity_fractions));
        }

        IloConstraintArray allowability_constraints(m_env);
        for (const auto& array : m_x) {
            IloExpr sum(m_env);
            for (IloInt i = 0; i < array.getSize(); ++i) {
                sum += array[i];
            }
            allowability_constraints.add(sum == 1.0);
        }

        IloConstraintArray balancing_constraints(m_env);
        for (size_t t = 0; t < prob.vehicles.size(); ++t) {
            int total_capacity = prob.vehicles[t].capacity;
            IloExpr sum(m_env);
            for (size_t i = 1; i < prob.customers.size(); ++i) {
                const auto& c = prob.customers[i];
                // TODO: the rest is questionable
                auto coeff = is_vehicle_allowed(i, t) * c.demand;
                for (IloInt j = 0; j < m_x[i-1].getSize(); ++j) {
                    sum += coeff * m_x[i-1][j];
                }
            }
            balancing_constraints.add(sum <= total_capacity * m_objective);
        }

        m_model.add(allowability_constraints);
        m_model.add(balancing_constraints);
    }

    /// 1 if vehicle is allowable, 0 otherwise. a[t][i]
    int is_vehicle_allowed(size_t customer, size_t vehicle) const {
        const auto& vehicles = m_prob.customers[customer].suitable_vehicles;
        bool allowed =  vehicles.cend() != std::find(vehicles.cbegin(),
            vehicles.cend(), static_cast<int>(vehicle));
        return static_cast<int>(allowed);
    }

    inline IloEnv& env() { return m_env; }
    inline std::vector<IloNumVarArray>& X() { return m_x; }
    inline IloModel& model() { return m_model; }
    inline IloCplex& algo() { return m_algo; }
    inline IloObjective& objective() { return m_objective; }
};
}  // anonymous

std::vector<Solution> cfrs_impl(const Problem& prob, size_t count) {
    Heuristic h(prob);
    return {};
}

}  // detail
}  // vrp
