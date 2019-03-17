#include "objective.h"

namespace vrp {
double objective(const Problem& prob, const Solution& sln) {
    double A = 1.;  // TODO(andrgolubev): add to CSV
    double objective_value = 0.;
    for (const auto& vehicle_route : sln.routes) {
        const auto& vehicle = prob.vehicles[vehicle_route.first];
        const auto& route = vehicle_route.second;
        for (auto i = route.cbegin(), j = std::next(i, 1); j != route.cend();
             ++i, ++j) {
            objective_value += vehicle.variable_cost * prob.costs[*i][*j];
            objective_value += A * prob.times[*i][*j];
        }
        objective_value += vehicle.fixed_cost;
    }
    return objective_value;
}
}  // namespace vrp
