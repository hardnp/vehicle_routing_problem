#include "parallel_insertion.h"
#include "logging.h"

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <tuple>

namespace vrp {
    namespace detail {
        std::vector<Solution> parallel_insertion(const Problem& prob,
                                                    size_t count) {
            std::vector<Solution> solution;

            // TODO: add for cycle for count > 1 (to run multiple tests)

            // amount of customers (without depo)
            const auto cust_amount = prob.customers.size - 1;
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
            int the_first_customer = range(r_gen);

            //find the biggest vehicle
            //int *biggest_veh_pointer;
            auto biggest_veh_pointer = std::max_element(prob.vehicles.begin(),
                    prob.vehicles.end(), [](const Vehicle &a,
                            const Vehicle &b){
                return a.capacity.volume < b.capacity.volume;
            });

            // sort customers by volumes
            std::sort(prob.customers.begin(), prob.customers.end(),
                    [](const Customer &a, const Customer &b){
                return a.demand.volume < b.demand.volume;
            });

            struct Splt_cust{
                unsigned int id;
                unsigned int ttl_volume;
                unsigned int lft_volume;
                const auto available_splits;
            };

            std::vector<Splt_cust> splited_customers;
            unsigned int i = 0;
            while(*biggest_veh_pointer < prob.customers[i].demand.volume){
                if(enable_splits) {
                    splited_customers.push_back({prob.customers[i].id,
                                                 prob.customers[i].demand.volume,
                                                 prob.customers[i].demand.volume,
                                                 max_splits
                    });
                    i++;
                }
                else {
                    std::cout << "No Solution (not enough splits)" << std::endl;
                    return 0;
                }
            }


            // sort customers by remoteness from depo
            std::sort(prob.times[0].begin(), prob.times[0].end(),
                    greater<int>());


            // sort vehicles by fixed_cost/volume
            std::sort(prob.vehicles.begin(), prob.vehicles.end(),
                      [](const Vehicle &a, const Vehicle &b){
                          return a.fixed_cost/a.capacity.volume <
                          b.fixed_cost/b.capacity.volume;
                      });

            // sum all customers demands size
            size_t demand_sum = 0;
            for(i = 1; i < points; i++){
                demand_sum += prob.customers[i].demand.value;
            }

            // find an amount of needed cars for the solution
            unsigned int needed_veh = 0;
            size_t cap_sum = 0;
            while(cap_sum < demand_sum){
                cap_sum += prob.vehicles[needed_veh].capacity.volume;
                needed_veh++;
            }

            // visited customers
            std::vector<size_t> visited_cust(points, 0);
            visited_cust[0] = 1;

            // used vehicles
            std::vector<size_t> used_veh(prob.vehicles.size(), 0);

            // list of routes (veh_id + customers ids)
            std::vector<std::pair<size_t, std::vector<size_t>>> routes;

            // choosing the first points for every route




            return solution;
        }
    }  // namespace detail
}  // namespace vrp