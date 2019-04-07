#include "savings.h"
#include <algorithm>
#include <array>
#include <ctime>
#include <iostream>
#include <map>
#include <tuple>

#define num 100

namespace vrp {
namespace detail {

std::vector<Solution> savings(const Problem& prob, size_t count) {
    const auto cust_size = prob.customers.size() - 1;
    const auto points = cust_size + 1;  // + 1 for depo itself

    // vehicle id <-> customets ids
    std::vector<std::pair<size_t, std::vector<size_t>>> routes;
    routes.reserve(cust_size);

    // valid routes
    std::vector<size_t> valid(cust_size, 1);
    valid[0] = 0;

    // visited customers
    std::vector<size_t> dest(points, 0);
    dest[0] = 1;

    // used veh
    std::vector<size_t> used_veh(prob.vehicles.size(), 0);

    // started from customers
    std::vector<size_t> src(points, 0);
    src[0] = 1;

    // backward edges
    std::vector<size_t> back_ed(points, 0);

    // route initialisation
    // for (unsigned int i = 0; i < points; ++i) routes.push_back({i, {0, i,
    // 0}});

    // cust id <-> route
    std::map<size_t, std::vector<size_t>*> route_map;

    // site dep veh <-> route
    std::vector<std::vector<int>> vehicles_for_route(points);
    vehicles_for_route[0] = {};

    for (unsigned int i = 1; i < points; ++i)
        vehicles_for_route[i] = prob.customers[i].suitable_vehicles;

    for (unsigned int i = 0; i < points; ++i) route_map[i] = &routes[i].second;

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
            save[i * points + j] = {
                i, j, prob.costs[i][0] + prob.costs[0][j] - prob.costs[i][j]};
        }
    }

    // sort and erase nonreq savings
    std::sort(save.begin(), save.end(),
              [](const S& a, const S& b) { return a.save_ij < b.save_ij; });

    save.erase(std::remove_if(save.begin(), save.end(),
                              [](const auto& o) {
                                  return (o.i == o.j || o.i == 0 || o.j == 0);
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
            prob.customers[i].hard_tw.first, prob.customers[i].hard_tw.second,
            std::max(prob.customers[i].hard_tw.first,
                     prob.times[0][prob.customers[i].id]),
            prob.customers[i].service_time};
    }

    std::vector<S> fine;
    int kk = 0;

    std::srand(time(0));
    bool pick_new_route = 1;

    // veh cust cap
    std::tuple<std::vector<int>, std::vector<size_t>, TransportationQuantity>
        current_route;
    int ff = 0;
    while (ff++ < 20) {
        if (pick_new_route) {
            //std::cout << "picking new one" << std::endl;
            for (size_t i = 1; i < points; ++i) {
                if (dest[i] == 0) {
                    current_route = {prob.customers[i].suitable_vehicles,
                                     {0, i, 0},
                                     prob.customers[i].demand};
                    dest[i] = 1;
                    pick_new_route = 0;
                    break;
                }
            }
            continue;
        } else {

            /*std::cout << "vehicles" << std::endl;
            for (auto& a : std::get<0>(current_route)) {
                std::cout << a << ' ';
            }
            std::cout << std::endl;

            std::cout << "cust" << std::endl;
            for (auto& a : std::get<1>(current_route)) {
                std::cout << a << ' ';
            }
            std::cout << std::endl;*/
            // std::cout << "demand " << std::get<2>(current_route) <<
            // std::endl;

            for (int i = save.size(); i > 0; --i) {
                auto best_save = save[save.size() - 1 - i];

                // add at the begin
                if (best_save.j == std::get<1>(current_route)[1] &&
                    !dest[best_save.i]) {

                    /*std::cout << "inserting in start " << best_save.i
                              << std::endl;*/

                    bool site_dep_viol = 0;
                    std::vector<int> common_veh = {};

                    for (auto& veh : vehicles_for_route[best_save.i]) {
                        if (std::find(std::get<0>(current_route).begin(),
                                      std::get<0>(current_route).end(),
                                      veh) != std::get<0>(current_route).end())
                            common_veh.push_back(veh);
                    }

                    if (common_veh.empty()) {
                        site_dep_viol = 1;
                    }

                    bool cap_viol = 0;
                    TransportationQuantity tmp_cap = std::get<2>(current_route);
                    tmp_cap += prob.customers[best_save.i].demand;

                    // veh is used or out of capacity
                    common_veh.erase(
                        std::remove_if(common_veh.begin(), common_veh.end(),
                                       [&](const int& x) {
                                           return ((prob.vehicles[x].capacity <
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

                    for (int s = 1; s < std::get<1>(current_route).size() - 1;
                         ++s) {
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
                            prob.costs[std::get<1>(current_route)
                                           [std::get<1>(current_route).size() -
                                            2]][0] >
                        times[0].finish) {
                        tw_violation = 1;
                    }

                    // magic num for to much waiting
                    if (times[std::get<1>(current_route)[1]].start -
                            times[std::get<1>(current_route)[1]].current +
                            offset >
                        num) {
                        tw_violation = 1;
                    }

                    if (!tw_violation && !site_dep_viol && !cap_viol) {
                        //std::cout << "is fine " << best_save.i << std::endl;
                        dest[best_save.i] = 1;
                        std::get<1>(current_route)
                            .insert(std::get<1>(current_route).begin() + 1,
                                    best_save.i);
                        std::get<2>(current_route) = tmp_cap;
                        std::get<0>(current_route) = common_veh;

                        for (int c = 1;
                             c < std::get<1>(current_route).size() - 1; ++c) {
                            times[std::get<1>(current_route)[c]]
                                .current = std::max(
                                times[std::get<1>(current_route)[c]].current +
                                    offset,
                                times[std::get<1>(current_route)[c]].start);
                        }
                    }

                    // add to the end
                } else if (best_save.i ==
                               std::get<1>(current_route)
                                   [std::get<1>(current_route).size() - 2] &&
                           !dest[best_save.j]) {
                   /* std::cout << "inserting in end " << best_save.j
                              << std::endl;*/
                    bool site_dep_viol = 0;
                    std::vector<int> common_veh = {};

                    for (auto& veh : vehicles_for_route[best_save.j]) {
                        if (std::find(std::get<0>(current_route).begin(),
                                      std::get<0>(current_route).end(),
                                      veh) != std::get<0>(current_route).end())
                            common_veh.push_back(veh);
                    }

                    if (common_veh.empty()) {
                        site_dep_viol = 1;
                    }

                    bool cap_viol = 0;
                    TransportationQuantity tmp_cap = std::get<2>(current_route);
                    tmp_cap += prob.customers[best_save.j].demand;

                    common_veh.erase(
                        std::remove_if(common_veh.begin(), common_veh.end(),
                                       [&](const int& x) {
                                           return ((prob.vehicles[x].capacity <
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
                    if (times[best_save.j].start - times[best_save.j].current +
                            offset >
                        num) {
                        tw_violation = 1;
                    }

                    if (!tw_violation && !site_dep_viol && !cap_viol) {
                        //std::cout << "is fine " << best_save.j << std::endl;
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
            //std::cout << "route ended" << std::endl;

            pick_new_route = 1;

            if (std::get<0>(current_route).empty()) {
                std::cout << "HIT THE END" << std::endl;
                break;
            }
            std::pair<size_t, std::vector<size_t>> route_to_add;
            route_to_add.first =
                std::get<0>(current_route)[std::get<0>(current_route).size()-1];
            used_veh[route_to_add.first] = 1;
            route_to_add.second = std::get<1>(current_route);
            routes.push_back(route_to_add);
        }
    }

    // main cycle
    /*while (kk < save.size()) {
      auto best_save = save[save.size() - 1 - kk++];

      // cust is not visited yet
      if (!src[best_save.i] && !dest[best_save.j] &&
          (best_save.j != best_save.i) &&
          !(back_ed[best_save.i] == best_save.j)) {
        // need to check offset for j-route
        int offset = times[best_save.i].current +
                     prob.times[best_save.i][best_save.j] -
                     times[best_save.j].current;

        // 0 stands for no site dependancy violations
        bool site_dep_viol = 0;
        std::vector<int> common_veh = {};

        for (auto &veh : vehicles_for_route[best_save.i]) {
          if (std::find(vehicles_for_route[best_save.j].begin(),
                        vehicles_for_route[best_save.j].end(),
                        veh) != vehicles_for_route[best_save.j].end())
            common_veh.push_back(veh);
        }

        if (common_veh.empty()) {
          site_dep_viol = 1;
        }

        // 0 stands for no capacity violations
        bool cap_viol = 0;
        TransportationQuantity tmp_cap = {0, 0};
        for (auto &a : (*route_map[best_save.i])) {
          tmp_cap += prob.customers[a].demand;
        }
        for (auto &a : (*route_map[best_save.j])) {
          tmp_cap += prob.customers[a].demand;
        }

        common_veh.erase(std::remove_if(common_veh.begin(), common_veh.end(),
                                        [&](const int &x) {
                                          return prob.vehicles[x].capacity <
                                                 tmp_cap;
                                        }),
                         common_veh.end());

        if (common_veh.empty()) {
          cap_viol = 1;
        }

        // 0 stands for no tw violations
        bool tw_violation = 0;

        // check time for the j-route
        auto prev = best_save.i;
        auto curr = best_save.j;
        int serv = 0;
        auto rt_sz = route_map[best_save.j]->size();
        for (int i = 1; i < rt_sz; ++i) {
          curr = (*route_map[best_save.j])[i];
          if (times[curr].current + offset + serv > times[curr].finish) {
            tw_violation = 1;
            break;
          }
          serv += times[prev].service_time;
          prev = curr;
        }

        if (!tw_violation && !site_dep_viol && !cap_viol) {
          dest[best_save.j] = 1;
          src[best_save.i] = 1;
          back_ed[best_save.j] = best_save.i;
          fine.push_back(best_save);
          valid[best_save.j] = 0;

          // set common vehicles
          for (int i = 1; i < route_map[best_save.i]->size() - 1; ++i)
            vehicles_for_route[(*route_map[best_save.i])[i]] = common_veh;
          for (int i = 1; i < route_map[best_save.j]->size() - 1; ++i)
            vehicles_for_route[(*route_map[best_save.j])[i]] = common_veh;

          prev = best_save.i;
          rt_sz = route_map[best_save.j]->size() - 1;
          auto cust_to_switch = (*route_map[best_save.j]);

          prev = best_save.i;
          curr = best_save.j;
          // customers to move to i-route
          for (int i = 1; i < rt_sz; ++i) {
            curr = cust_to_switch[i];
            times[curr].current =
                std::max(times[curr].current + offset +
    times[prev].service_time, times[curr].start);

            prev = curr;
            route_map[best_save.i]->insert(route_map[best_save.i]->end() - 1,
                                           curr);
            route_map[curr] = route_map[best_save.i];
          }
        }
      }
    }*/

    std::cout << std::endl << "final routes" << std::endl;
    // TODO: consider more generic approach
    /*int route_num = 0;
    int tmp = 0;
    for (auto a : routes) {
      if (valid[tmp++]) {
        route_num++;
        std::cout << "vehicles:" << std::endl;
        for (auto b : vehicles_for_route[a.second[1]]) {
          std::cout << b << ' ';
        }
        std::cout << std::endl;
        for (auto b : a.second) {
          std::cout << b << ' ' << times[b].current << ' ';
        }
        std::cout << std::endl;
      }
    }
    std::cout << "SIZE IS " << route_num << std::endl;*/
    for (auto a : routes) {
        std::cout << "vehicle " << a.first << std::endl;
        for (auto b : a.second) {
            std::cout << b << ' ';
        }
        std::cout << std::endl << std::endl;
    }

    // after all
    // put here all the solution stuff
    return {};
}  // namespace detail
}  // namespace detail
}  // namespace vrp
