#include "savings.h"
#include <algorithm>
#include <array>
#include <ctime>
#include <iostream>
#include <map>
#include <numeric>
#include <tuple>

// some regularisation
#define MAX_WAITING_TIME 1000
#define MAX_TIME_VIOLATION 0
#define MAX_CAPACITY_VIOLATION 0
#define FIRST_RAND_ROUTES 10

namespace vrp {
namespace detail {

std::vector<Solution> savings(const Problem& prob, size_t count) {
    std::vector<Solution> solutions;
    std::srand(5489u);

    for (size_t it = 0; it < count; ++it) {
        const auto cust_size = prob.customers.size() - 1;
        const auto points = cust_size + 1;  // + 1 for depo itself

        // vehicle id <-> customets ids
        std::vector<std::pair<size_t, std::vector<size_t>>> routes;
        routes.reserve(cust_size);

        // visited customers
        std::vector<size_t> dest(points, 0);
        dest[0] = 1;

        // used vehicles
        std::vector<size_t> used_veh(prob.vehicles.size(), 0);

        // site dep veh <-> route
        std::vector<std::vector<int>> vehicles_for_cust(points);
        vehicles_for_cust[0] = {};

        // FIXME: generation via iota for solomon
        for (unsigned int i = 1; i < points; ++i) {
            vehicles_for_cust[i].resize(prob.vehicles.size());
            std::iota(std::begin(vehicles_for_cust[i]),
                      std::end(vehicles_for_cust[i]), 0);
        }
        // vehicles_for_cust[i] = prob.customers[i].suitable_vehicles;

        // saving for i -> j edge
        struct S {
            unsigned int i;
            unsigned int j;
            double save_ij;
        };

        std::vector<S> save(points * points, {0, 0, 0});

        // calculating matrix(vector) of savings
        for (unsigned int i = 0; i < points; ++i) {
            for (unsigned int j = 0; j < points; ++j) {
                save[i * points + j] = {i, j,
                                        prob.costs[i][0] + prob.costs[0][j] -
                                            prob.costs[i][j]};
            }
        }

        // sort and erase nonreq savings
        std::sort(save.begin(), save.end(), [](const S& a, const S& b) {
            return a.save_ij - b.save_ij > 0;
        });

        save.erase(std::remove_if(save.begin(), save.end(),
                                  [](const auto& o) {
                                      return (o.i == o.j || o.i == 0 ||
                                              o.j == 0);
                                  }),
                   save.end());

        // times for customer
        struct Time {
            int start;
            int finish;
            int current;
            int service_time;
        };

        // customer id <-> hard tw
        std::map<size_t, Time> times;

        // initial times
        for (size_t i = 0; i < points; ++i) {
            times[prob.customers[i].id] = {
                prob.customers[i].hard_tw.first,
                prob.customers[i].hard_tw.second,
                std::max(prob.customers[i].hard_tw.first,
                         prob.times[0][prob.customers[i].id]),
                prob.customers[i].service_time};
        }

        // veh cust cap
        std::tuple<std::vector<int>, std::vector<size_t>,
                   TransportationQuantity>
            current_route;

        size_t rand_cust = rand() % cust_size + 1;
        current_route = {vehicles_for_cust[rand_cust],
                         {0, rand_cust, 0},
                         prob.customers[rand_cust].demand};

        // number of visited customers
        size_t served = 1;
        dest[rand_cust] = 1;
        bool pick_new_route = 0;
        int first_rand_routes = 1;

        while (served < cust_size) {

            if (pick_new_route) {
                // initialise route with random customer
                if (first_rand_routes < FIRST_RAND_ROUTES) {
                    while (true) {
                        size_t rand_cust = rand() % cust_size + 1;
                        if (dest[rand_cust] == 1)
                            continue;
                        else {
                            ++first_rand_routes;
                            current_route = {vehicles_for_cust[rand_cust],
                                             {0, rand_cust, 0},
                                             prob.customers[rand_cust].demand};
                            dest[rand_cust] = 1;
                            pick_new_route = 0;
                            ++served;
                            break;
                        }
                    }
                    // find first non-visited
                } else {
                    for (size_t i = 1; i < points; ++i) {
                        if (dest[i] == 0) {
                            current_route = {vehicles_for_cust[i],
                                             {0, i, 0},
                                             prob.customers[i].demand};
                            dest[i] = 1;
                            pick_new_route = 0;
                            ++served;
                            break;
                        }
                    }
                }

                // workaround if the last customer is served randomly
                if (served == cust_size) {
                    std::pair<size_t, std::vector<size_t>> route_to_add;
                    route_to_add.first = std::get<0>(current_route)[0];
                    used_veh[route_to_add.first] = 1;
                    route_to_add.second = std::get<1>(current_route);
                    routes.push_back(route_to_add);
                }

                continue;

            } else {

                for (size_t i = 0; i < save.size(); ++i) {
                    auto best_save = save[i];

                    // add to the begin (0 -> i -> current route -> 0)
                    if (best_save.j == std::get<1>(current_route)[1] &&
                        !dest[best_save.i]) {

                        bool site_dep_viol = 0;
                        std::vector<int> common_veh = {};

                        for (auto& veh : vehicles_for_cust[best_save.i]) {
                            if (std::find(std::get<0>(current_route).begin(),
                                          std::get<0>(current_route).end(),
                                          veh) !=
                                std::get<0>(current_route).end())
                                common_veh.push_back(veh);
                        }

                        if (common_veh.empty()) {
                            site_dep_viol = 1;
                        }

                        bool cap_viol = 0;
                        TransportationQuantity tmp_cap =
                            std::get<2>(current_route);
                        tmp_cap += prob.customers[best_save.i].demand;

                        // veh is used or out of capacity
                        common_veh.erase(
                            std::remove_if(common_veh.begin(), common_veh.end(),
                                           [&](const int& x) {
                                               return (
                                                   (prob.vehicles[x].capacity +
                                                        MAX_CAPACITY_VIOLATION <
                                                    tmp_cap) ||
                                                   (used_veh[x] == 1));
                                           }),
                            common_veh.end());

                        if (common_veh.empty()) {
                            cap_viol = 1;
                        }

                        bool tw_violation = 0;

                        int offset = times[best_save.i].current +
                                     times[best_save.i].service_time +
                                     prob.times[best_save.i][best_save.j] -
                                     times[best_save.j].current;

                        // do NOT remove the &
                        auto& current_customers = std::get<1>(current_route);

                        auto prev = best_save.i;
                        auto curr = best_save.j;
                        int curr_time = times[best_save.i].current +
                                        times[best_save.i].service_time +
                                        prob.times[best_save.i][best_save.j];
                        for (size_t s = 1; s < current_customers.size() - 1;
                             ++s) {
                            if (curr_time + times[curr].service_time >
                                times[curr].finish + MAX_TIME_VIOLATION) {
                                tw_violation = 1;
                                break;
                            }
                            prev = curr;
                            curr = current_customers[s + 1];
                            curr_time =
                                std::max(curr_time + times[prev].service_time +
                                             prob.times[prev][curr],
                                         times[curr].start);
                        }

                        if (times[current_customers[current_customers.size() -
                                                    2]]
                                    .current +
                                times[current_customers
                                          [current_customers.size() - 2]]
                                    .service_time +
                                offset +
                                prob.times[current_customers
                                               [current_customers.size() - 2]]
                                          [0] >
                            times[0].finish + MAX_TIME_VIOLATION) {
                            tw_violation = 1;
                        }

                        // magic num for to much waiting
                        if (times[current_customers[1]].start -
                                times[current_customers[1]].current + offset >
                            MAX_WAITING_TIME) {
                            tw_violation = 1;
                        }

                        // adding to the begin
                        if (!tw_violation && !site_dep_viol && !cap_viol) {
                            ++served;
                            dest[best_save.i] = 1;
                            current_customers.insert(
                                current_customers.begin() + 1, best_save.i);
                            std::get<2>(current_route) = tmp_cap;
                            std::get<0>(current_route) = common_veh;

                            auto prev = best_save.i;
                            auto curr = best_save.j;
                            int curr_time =
                                times[best_save.i].current +
                                times[best_save.i].service_time +
                                prob.times[best_save.i][best_save.j];
                            for (size_t c = 1; c < current_customers.size() - 1;
                                 ++c) {
                                times[curr].current =
                                    std::max(curr_time, times[curr].current);
                                prev = curr;
                                curr = current_customers[c + 1];
                                curr_time = std::max(
                                    curr_time + times[prev].service_time +
                                        prob.times[prev][curr],
                                    times[curr].start);
                            }
                        }

                        // add to the end (0 -> current route -> j -> 0)
                    } else if (best_save.i ==
                                   std::get<1>(
                                       current_route)[std::get<1>(current_route)
                                                          .size() -
                                                      2] &&
                               !dest[best_save.j]) {

                        bool site_dep_viol = 0;
                        std::vector<int> common_veh = {};

                        for (auto& veh : vehicles_for_cust[best_save.j]) {
                            if (std::find(std::get<0>(current_route).begin(),
                                          std::get<0>(current_route).end(),
                                          veh) !=
                                std::get<0>(current_route).end())
                                common_veh.push_back(veh);
                        }

                        if (common_veh.empty()) {
                            site_dep_viol = 1;
                        }

                        bool cap_viol = 0;
                        TransportationQuantity tmp_cap =
                            std::get<2>(current_route);
                        tmp_cap += prob.customers[best_save.j].demand;

                        common_veh.erase(
                            std::remove_if(common_veh.begin(), common_veh.end(),
                                           [&](const int& x) {
                                               return (
                                                   (prob.vehicles[x].capacity +
                                                        MAX_CAPACITY_VIOLATION <
                                                    tmp_cap) ||
                                                   (used_veh[x] == 1));
                                           }),
                            common_veh.end());

                        if (common_veh.empty()) {
                            cap_viol = 1;
                        }

                        bool tw_violation = 0;

                        int offset = times[best_save.i].current +
                                     times[best_save.i].service_time +
                                     prob.times[best_save.i][best_save.j] -
                                     times[best_save.j].current;

                        if (times[best_save.j].current + offset +
                                times[best_save.j].service_time >
                            times[best_save.j].finish + MAX_TIME_VIOLATION) {
                            tw_violation = 1;
                        }

                        if (times[best_save.j].current + offset +
                                prob.times[best_save.j][0] +
                                times[best_save.j].service_time >
                            times[0].finish + MAX_TIME_VIOLATION) {
                            tw_violation = 1;
                        }

                        // magic num for to much waiting
                        if (times[best_save.j].start -
                                times[best_save.j].current + offset >
                            MAX_WAITING_TIME) {
                            tw_violation = 1;
                        }

                        if (!tw_violation && !site_dep_viol && !cap_viol) {
                            // adding to the end
                            ++served;
                            dest[best_save.j] = 1;
                            std::get<1>(current_route)
                                .insert(std::get<1>(current_route).end() - 1,
                                        best_save.j);
                            std::get<2>(current_route) = tmp_cap;
                            std::get<0>(current_route) = common_veh;
                            times[best_save.j].current =
                                std::max(times[best_save.j].current + offset,
                                         times[best_save.j].current);
                        }
                    }
                }

                pick_new_route = 1;

                std::pair<size_t, std::vector<size_t>> route_to_add;
                route_to_add.first = std::get<0>(current_route)[0];
                used_veh[route_to_add.first] = 1;
                route_to_add.second = std::get<1>(current_route);
                routes.push_back(route_to_add);
            }
        }

        Solution sav_sol;
        std::vector<std::pair<size_t, std::list<size_t>>> listed_routes(
            routes.size());
        int ii = 0;
        for (auto a : routes) {
            listed_routes[ii].first = a.first;
            for (auto b : a.second) {
                listed_routes[ii].second.push_back(b);
            }
            ++ii;
        }
        sav_sol.routes = listed_routes;

        // update solution info
        sav_sol.update_customer_owners(prob);
        sav_sol.update_times(prob);
        sav_sol.update_used_vehicles();

        solutions.push_back(sav_sol);
    }

    return solutions;
}

}  // namespace detail
}  // namespace vrp
