#include "savings.h"
#include <algorithm>
#include <array>
#include <ctime>
#include <iostream>
#include <map>
#include <tuple>
#include <numeric>

#define num 150

namespace vrp {
namespace detail {

std::vector<Solution> savings(const Problem& prob, size_t count) {
    std::srand(time(0));
    for (int it = 0; it < 1; ++it) {
        const auto cust_size = prob.customers.size() - 1;
        const auto points = cust_size + 1;  // + 1 for depo itself

        // vehicle id <-> customets ids
        std::vector<std::pair<size_t, std::vector<size_t>>> routes;
        routes.reserve(cust_size);

        // visited customers
        std::vector<size_t> dest(points, 0);
        dest[0] = 1;

        // used veh
        std::vector<size_t> used_veh(prob.vehicles.size(), 0);

        // site dep veh <-> route
        std::vector<std::vector<int>> vehicles_for_cust(points);
        vehicles_for_cust[0] = {};

        for (unsigned int i = 1; i < points; ++i){
            vehicles_for_cust[i].resize(prob.vehicles.size());
            std::iota(std::begin(vehicles_for_cust[i]),
                      std::end(vehicles_for_cust[i]), 0);
        }
        //vehicles_for_cust[i] = prob.customers[i].suitable_vehicles;

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
        std::sort(save.begin(), save.end(),
                  [](const S& a, const S& b) { return a.save_ij < b.save_ij; });

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
        for (auto i = 0; i < points; ++i) {
            times[prob.customers[i].id] = {
                prob.customers[i].hard_tw.first,
                prob.customers[i].hard_tw.second,
                std::max(prob.customers[i].hard_tw.first,
                         prob.times[0][prob.customers[i].id]),
                prob.customers[i].service_time};
        }

        bool pick_new_route = 0;

        // veh cust cap
        std::tuple<std::vector<int>, std::vector<size_t>,
                   TransportationQuantity>
            current_route;

        size_t rand_cust = rand() % cust_size + 1;
        current_route = {vehicles_for_cust[rand_cust]
            /*prob.customers[rand_cust].suitable_vehicles*/,
                         {0, rand_cust, 0},
                         prob.customers[rand_cust].demand};

        int served = 0;

        while (served < cust_size) {

            if (pick_new_route) {
                for (size_t i = 1; i < points; ++i) {
                    if (dest[i] == 0) {
                        current_route = {
                            vehicles_for_cust[i] /*prob.customers[i]
                                .suitable_vehicles*/,
                                         {0, i, 0},
                                         prob.customers[i].demand};
                        /*std::cout << "picking new route with " << i
                                  << std::endl;*/
                        dest[i] = 1;
                        pick_new_route = 0;
                        ++served;
                        break;
                    }
                }
                continue;

            } else {

                for (int i = save.size(); i > 0; --i) {
                    auto best_save = save[save.size() - 1 - i];

                    // add at the begin
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
                                                   (prob.vehicles[x].capacity <
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

                        for (int s = 1;
                             s < std::get<1>(current_route).size() - 1; ++s) {
                            if (times[std::get<1>(current_route)[s]].current +
                                    offset >
                                times[std::get<1>(current_route)[s]].finish) {
                                tw_violation = 1;
                                break;
                            }
                        }

                        if (times[std::get<1>(current_route)
                                      [std::get<1>(current_route).size() - 2]]
                                    .current +
                                offset +
                                prob.costs[std::get<1>(
                                    current_route)[std::get<1>(current_route)
                                                       .size() -
                                                   2]][0] >
                            times[0].finish) {
                            tw_violation = 1;
                        }

                        // magic num for to much waiting
                        /*if (times[std::get<1>(current_route)[1]].start -
                                times[std::get<1>(current_route)[1]].current +
                                offset >
                            num) {
                            tw_violation = 1;
                        }*/

                        if (!tw_violation && !site_dep_viol && !cap_viol) {
                           /* std::cout << "insertion is start " << best_save.i
                                      << std::endl;*/
                            ++served;
                            dest[best_save.i] = 1;
                            std::get<1>(current_route)
                                .insert(std::get<1>(current_route).begin() + 1,
                                        best_save.i);
                            std::get<2>(current_route) = tmp_cap;
                            std::get<0>(current_route) = common_veh;

                            for (int c = 1;
                                 c < std::get<1>(current_route).size() - 1;
                                 ++c) {
                                times[std::get<1>(current_route)[c]]
                                    .current = std::max(
                                    times[std::get<1>(current_route)[c]]
                                            .current +
                                        offset,
                                    times[std::get<1>(current_route)[c]].start);
                            }
                        }

                        // add to the end
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
                                                   (prob.vehicles[x].capacity <
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

                        if (times[best_save.j].current + offset >
                            times[best_save.j].finish) {
                            tw_violation = 1;
                        }

                        if (times[best_save.j].current + offset +
                                prob.costs[best_save.j][0] >
                            times[0].finish) {
                            tw_violation = 1;
                        }

                        // magic num for to much waiting
                        /*if (times[best_save.j].start -
                        times[best_save.j].current + offset > num) {
                        tw_violation = 1;
                        }*/

                        if (!tw_violation && !site_dep_viol && !cap_viol) {
                           /* std::cout << "insertion to end " << best_save.j
                                      << std::endl;*/
                            ++served;
                            dest[best_save.j] = 1;
                            std::get<1>(current_route)
                                .insert(std::get<1>(current_route).end() - 1,
                                        best_save.j);
                            std::get<2>(current_route) = tmp_cap;
                            std::get<0>(current_route) = common_veh;
                            times[best_save.j].current =
                                std::max(times[best_save.j].current + offset,
                                         times[best_save.j].start);
                        }
                    }
                }
                /*std::cout << "route ended" << std::endl;

                std::cout << "vehicles" << std::endl;
                for (auto& a : std::get<0>(current_route)) {
                    std::cout << a << ' ';
                }
                std::cout << std::endl;

                std::cout << "cust" << std::endl;
                for (auto& a : std::get<1>(current_route)) {
                    std::cout << a << ' ';
                }
                std::cout << std::endl;*/

                pick_new_route = 1;

                std::pair<size_t, std::vector<size_t>> route_to_add;
                /*route_to_add.first = std::get<0>(
                    current_route)[std::get<0>(current_route).size() - 1];*/
                route_to_add.first = std::get<0>(
                    current_route)[0];
                used_veh[route_to_add.first] = 1;
                route_to_add.second = std::get<1>(current_route);
                routes.push_back(route_to_add);
            }
        }

        std::cout << std::endl << "final routes" << std::endl;

        for (auto a : routes) {
            std::cout << "vehicle " << a.first << std::endl;
            for (auto b : a.second) {
                std::cout << b << ' ';
            }
            std::cout << std::endl << std::endl;
        }
    }

    // after all
    // put here all the solution stuff
    return {};
}  // namespace detail
}  // namespace detail
}  // namespace vrp
