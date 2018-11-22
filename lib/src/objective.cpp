#include "objective.h"

namespace vrp {
double objective(const Problem& prob, const Solution& sln) {
    double A = 1.;  // TODO(agolubev): add to CSV
    double objective_value = 0.;
    for (const auto& route : sln.routes) {
        const auto& vehicle = prob.vehicles[route.first];
        const auto& customers = route.second;
        for (size_t i = 0; i < customers.size() - 1; ++i) {
            objective_value += vehicle.variable_cost
                * prob.costs[customers[i]][customers[i+1]];
            objective_value += A * prob.times[customers[i]][customers[i+1]];
        }
        objective_value += vehicle.fixed_cost;
    }
    return objective_value;
}
}  // vrp
