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

            // amount of customers (without depo)
            const auto cust_amount = prob.customers.size - 1;

            // enable spits and available amount of them
            const auto enable_splits = prob.enable_splits();
            auto max_splits = prob.max_splits;

            // a random number generator
            std::mt19937 r_gen;

            // two different ranges for it
            std::uniform_int_distribution<int> range(0,2);
            //std::uniform_int_distribution<int> range(0,4);

            // a random number itself
            int the_first_customer = range(r_gen);

            //find the biggest vehicle
            int *biggest_veh_pointer; /// try to set as auto below if wouldn't work
            biggest_veh_pointer = std::max_element(prob.vehicles.begin(),
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
            };

            std::vector<Splt_cust> splited_customers;

            i = 0;
            while(*biggest_veh_pointer < prob.customers[i].demand.volume){
                if(max_splits > 0){
                    splited_customers.push_back({prob.customers.id,
                                                 prob.customers.demand.volume,
                                                 prob.customers.demand.volume});
                    max_splits--;
                    i++;
                }
                else{
                    cout << "No Solution (not enough splits)" << endl;
                    return 0;
                }
            }


            // sort customers by remoteness from depo
            // TODO!
            std::sort(prob.customers.begin(), prob.customers.end(),
                      [](const Customer &a, const Customer &b){
                          return a.demand.volume < b.demand.volume;
                      });












            return solution;
        }
    }  // namespace detail
}  // namespace vrp