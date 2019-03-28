#include "objective.h"

namespace vrp {

double objective(const Problem& prob, const Solution& sln) {
    double objective_value = 0.;
    for (const auto& vehicle_route : sln.routes) {
        const auto& vehicle = prob.vehicles[vehicle_route.first];
        const auto& route = vehicle_route.second;
        for (auto i = route.cbegin(), j = std::next(i); j != route.cend();
             ++i, ++j) {
            objective_value += vehicle.variable_cost * prob.costs[*i][*j];
            objective_value += prob.time_coeff * prob.times[*i][*j];
        }
        objective_value += vehicle.fixed_cost;
    }
    return objective_value;
}

double objective(const Problem& prob, Solution::VehicleIndex vi,
                 const Solution::RouteType& route) {
    double objective_value = 0.;
    const auto& vehicle = prob.vehicles[vi];
    for (auto i = route.cbegin(), j = std::next(i); j != route.cend();
         ++i, ++j) {
        objective_value += vehicle.variable_cost * prob.costs[*i][*j];
        objective_value += prob.time_coeff * prob.times[*i][*j];
    }
    objective_value += vehicle.fixed_cost;
    return objective_value;
}

double cost_function(const Problem& prob, const Solution& sln) {
    double cost = 0.;
    for (const auto& vehicle_route : sln.routes) {
        const auto& route = vehicle_route.second;
        for (auto i = route.cbegin(), j = std::next(i); j != route.cend();
             ++i, ++j) {
            cost += prob.costs[*i][*j];
        }
    }
    return cost;
}
}  // namespace vrp
