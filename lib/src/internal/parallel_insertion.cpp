#include "parallel_insertion.h"
#include "logging.h"

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <tuple>

namespace vrp {
    namespace detail {

        // a solution itself
        std::vector<Solution> solution;

        namespace {
            // a function to calculate the solution
            bool find_the_solution (const Problem& probl, unsigned int v_needed,
                                    std::vector<std::pair<Vehicle,
                                    bool> > vehicles_lcl,
                                    std::vector<std::tuple<Customer, size_t,
                                    bool,size_t,
                                    std::vector<int>>> customers_lcl,
                                    size_t the_firts_cust){

                // check if solved successfully
                bool check = true;

                // routes of solution representation
                /// first -veh numb, second[i].first -cust numb, second[i].second -route point times
                std::vector<std::pair<size_t, std::vector<std::pair<size_t,
                                            RoutePointTime>>>> routes(v_needed);

                // 0 route 0 customer add
                routes[0].second[0].first = the_firts_cust;
                routes[0].second[0].second.arrive =
                        get<1>(customers_lcl[the_firts_cust]);
                if(routes[0].second[0].second.arrive <
                            get<0>(customers_lcl[the_firts_cust]).hard_tw.first)
                {
                    routes[0].second[0].second.start =
                            get<0>(customers_lcl[the_firts_cust]).hard_tw.first;
                    routes[0].second[0].second.finish =
                            routes[0].second[0].second.start +
                            get<0>(customers_lcl[the_firts_cust]).service_time;
                }
                else {
                    routes[0].second[0].second.start =
                            routes[0].second[0].second.arrive;
                    routes[0].second[0].second.finish =
                            routes[0].second[0].second.start +
                            get<0>(customers_lcl[the_firts_cust]).service_time;
                }

                get<3>(customers_lcl[the_firts_cust]) = 1;

                // set 0-points for each route
                size_t stc = 0;
                size_t stc_max = 0;
                size_t c_num;
                // fi - current route in work number
                for(unsigned int fi=1; fi < v_needed; fi++){
                    stc = 0;
                    stc_max = 0;
                    // fii - number of customer in work
                    for(unsigned int fii=1; fii < customers_lcl.size();fii++){
                        // if not in route
                        if(get<3>(customers_lcl[fii]) == 0){
                            unsigned int fij = 0;
                            // sum all times to all routed customers
                            while(fij<fi){
                                size_t cstmr = routes[fij].second[0].first;
                                stc += prob.times[fii][cstmr];
                                ++fij;
                            }
                            // add time to depo to sum
                            stc += get<1>(customers_lcl[fii]);
                            // check if this sum is max
                            if(stc_max < stc){
                                stc_max = stc;
                                c_num = fii;
                            }
                        }
                    }
                    // add in route[fi] a cust with the max sum
                    routes[fi].second[0].first = c_num;
                    routes[fi].second[0].second.arrive =
                            get<1>(customers_lcl[c_num]);
                    if(routes[fi].second[0].second.arrive <
                    get<0>(customers_lcl[c_num]).hard_tw.first)
                    {
                        routes[fi].second[0].second.start =
                                get<0>(customers_lcl[c_num]).hard_tw.first;
                        routes[fi].second[0].second.finish =
                                routes[fi].second[0].second.start +
                                get<0>(customers_lcl[c_num]).service_time;
                    }
                    else {
                        routes[fi].second[0].second.start =
                                routes[fi].second[0].second.arrive;
                        routes[fi].second[0].second.finish =
                                routes[fi].second[0].second.start +
                                get<0>(customers_lcl[c_num]).service_time;
                    }
                    get<3>(customers_lcl[c_num]) = 1;
                }

                // at this moment we got all 0-point in all routes



                if (check) {return true;}
                else {return false;}
            }

        } // namespace



        std::vector<Solution> parallel_insertion(const Problem& prob,
                                                    size_t count) {

            // TODO: add for() cycle for count > 1 (to run multiple tests)

            // amount of customers (without depo)
            const auto cust_amount = prob.customers.size() - 1;
            const auto points = cust_amount + 1;

            // enable spits and available amount of them for each customer
            const auto enable_splits = prob.enable_splits();
            const auto max_splits = prob.max_splits;

            // a random number generator
            std::mt19937 r_gen;

            // two different ranges for it
            std::uniform_int_distribution<int> range(0,2);
            //std::uniform_int_distribution<int> range(0,4);

            // a random number itself
            size_t the_first_customer = range(r_gen);

            // customers local representation
            /// 0 -Customer, 1 -time to depo, 2 -split, 3 -visited, 4 -route number
            std::vector<std::tuple<Customer, size_t, bool,
                                    size_t,
                                    std::vector<int>(max_splits, -1)
                                            > > customers_local(points);
            for(unsigned int cn=0; cn < points; cn++){
                get<0>(customers_local[cn]) = prob.customers[cn];
                get<1>(customers_local[cn]) = prob.times[0][cn];
                get<2>(customers_local[cn]) = false;
                get<3>(customers_local[cn]) = 0;
            }
            // set depo as visited
            get<3>(customers_local[0]) = 1;

            // vehicles local representation
            /// first -Vehicle, second -used
            std::vector<std::pair<Vehicle,
                                  bool> > vehicles_local(prob.vehicles.size());
            for(unsigned int vn=0; vn < prob.vehicles.size(); vn++){
                vehicles_local[vn].first = prob.vehicles[vn];
                vehicles_local[vn].second = false;
            }

            //find the biggest vehicle
            auto biggest_veh_pointer = std::max_element(vehicles_local.begin(),
                    vehicles_local.end(), [](const std::pair<Vehicle, bool> &a,
                            const std::pair<Vehicle, bool> &b){
                        return (a.capacity.volume < b.capacity.volume);
                    });

            // sort customers by volumes
            std::sort(customers_local.begin(), customers_local.end(),
                    [](const tuple<Customer, size_t, bool, size_t,
                            std::vector<int>(max_splits, -1)> &a,
                            const tuple<Customer, size_t, bool, size_t,
                            std::vector<int>(max_splits, -1)> &b){
                return (get<0>(a).demand.volume > get<0>(b).demand.volume);
            });

            unsigned int i = 0;
            while(*biggest_veh_pointer < customers_local[i].demand.volume){
                if(enable_splits) {
                    get<2>(customers_local[i]) = true;
                    i++;
                }
                else {
                    std::cout << "No Solution (no splits)" << std::endl;
                    return 0;
                }
            }

            // sort customers by remoteness from depo
            std::sort(customers_local.begin(), customers_local.end(),
                      [](const tuple<Customer, size_t, bool, size_t,
                              std::vector<int>(max_splits, -1)> &a,
                         const tuple<Customer, size_t, bool, size_t,
                                 std::vector<int>(max_splits, -1)> &b){
                          return (get<1>(a) > get<1>(b));
                      });



            // sort vehicles by fixed_cost/volume
            std::sort(vehicles_local.begin(), vehicles_local.end(),
                      [](const std::pair<Vehicle,bool> &a,
                              const std::pair<Vehicle,bool> &b){
                          return (a.first.fixed_cost/a.first.capacity.volume <
                          b.first.fixed_cost/b.first.capacity.volume);
                      });

            // sum all customers demands size
            size_t demand_sum = 0;
            for(i = 1; i < points; i++){
                demand_sum += get<0>(customers_local[i]).demand.volume;
            }

            // find an amount of needed cars for the solution
            unsigned int needed_veh = 0;
            size_t capacity_sum = 0;
            while(capacity_sum < demand_sum){
                capacity_sum += vehicles_local[needed_veh].first.capacity.volume;
                needed_veh++;
            }

            /// calling a function which will try to find a solution
            if (!find_the_solution(prob, needed_veh, vehicles_local,
                    customers_local, the_first_customer)) {
                // if not - increment an amount of used vehicles
                needed_veh++;
                find_the_solution(prob, needed_veh, vehicles_local,
                                  customers_local, the_first_customer);
            }
            else {return solution;}
        }
    }  // namespace detail
}  // namespace vrp