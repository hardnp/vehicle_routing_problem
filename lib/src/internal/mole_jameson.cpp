#include "logging.h"
#include "mole_jameson.h"

#include "problem.h"
#include "solution.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
#include <tuple>
#include <vector>
#include <type_traits>

namespace utility {
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value, T>, typename U = std::mt19937>
    T random(T upper_bound = std::numeric_limits<T>::max()) {
        static U generator { std::random_device{}() };
        return generator() % upper_bound;
    }
}

/** TODO:
 * Split-delivery, site-dependency.
 * Correct insertion validation functions should be transferred ASAP.
 * */

namespace vrp {
    namespace detail {

        std::vector<Solution> mole_jameson(const Problem& prob, size_t count) {

            using namespace utility;

            struct CustomerStatus {
                bool is_routed;
                std::int32_t current_load;
                double departure_time;
            };

            struct InsertionCandidate {
                Solution::RouteType::const_iterator position;
                double badness, goodness;
                size_t id;
            };

            /** This tracker structure allows to store and access the customer's data, which is related to heuristics, in the convenient way.
             *  TODO: This is probably redundant. Should have used the customer's pre-defined internals. */
            class CustomerTracker {
            public:
                CustomerTracker() = delete;
                CustomerTracker(Problem const& prob):
                    customers {prob.customers}, statuses {prob.customers.size()}, costs {prob.costs} {}
                CustomerTracker(CustomerTracker const&) = delete;
                CustomerTracker(CustomerTracker&&) = delete;

                auto is_routed          (size_t id) const { return statuses[id].is_routed;      }
                auto get_current_load   (size_t id) const { return statuses[id].current_load;   }
                auto get_departure_time (size_t id) const { return statuses[id].departure_time; }

                auto update(Solution::RouteType const& route, Solution::RouteType::const_iterator iterator) -> void {
                    auto iter = iterator, prev = std::prev(iter);
                    statuses[*iter].is_routed = 1;
                    for (; *iter != 0 ;) {
                        auto& current_customer = customers[*iter];
                        auto& previous_customer = customers[*prev];
                        auto& current_status = statuses[current_customer.id];
                        auto& previous_status = statuses[previous_customer.id];
                        current_status.departure_time = 1; /** TODO: ... */
                        current_status.current_load = previous_status.current_load + current_customer.demand;
                        prev = iter++;
                    }
                }

            private:
                std::vector<Customer> const& customers;
                std::vector<CustomerStatus> statuses;
                decltype(prob.costs) const& costs;
            };

            std::vector<Solution> solution;
            Solution current_solution;
            /** @note L/M coefficients defined by the Mole-Jameson Heuristic. */
            static constexpr auto LAMBDA = 1.1;
            static constexpr auto MU = 1.1;

            /** @note The badness (alpha) and goodness (beta) ratings for the insertion. */
            const auto alpha_evaluation = [&](size_t i, size_t k, size_t j) {
                return prob.costs[i][k] + prob.costs[k][j] - LAMBDA * prob.costs[i][j];
            };

            const auto beta_evaluation = [&](size_t i, size_t k, size_t j) {
                return MU * prob.costs[0][k] - alpha_evaluation(i, k, j);
            };

            const auto next_departure_time = []() {
                auto departure = 0.0;
                return departure;
            };

            /** @note Check whether it's possible to insert the customer in the intermediate route. */
            auto get_insertion_feasiblity = [&]
                    (Solution::RouteType const& route, CustomerTracker const& tracker,
                     Customer const& customer, Solution::RouteType::const_iterator position) {
                --position;
                const auto* previous_customer = &prob.customers[*position];
                const auto* current_customer = &customer;
                auto departure_time = tracker.get_departure_time(previous_customer->id);
                auto current_load = tracker.get_current_load(previous_customer->id);

                for (;;) {
                    const auto required = departure_time + prob.costs[previous_customer->id][current_customer->id];
                    if (required > current_customer->hard_tw.first) return false;
                    departure_time = 0; /** TODO: Transfer the departure time calculation. */
                    if (current_load += current_customer->demand > capacity) return false;
                    if (++position == std::cend(route)) break;
                    previous_customer = current_customer;
                    current_customer = &prob.customers[*position];
                }
                return true;
            };

            CustomerTracker tracker(prob);
            for (std::size_t iteration = 0; iteration < prob.n_vehicles(); ++iteration) {
            #ifdef MOLE_JAMESON_RANDOMIZED
                std::vector<Customer> unrouted;
                for (std::size_t index = 0; index < prob.customers.size(); ++index) {
                    if (!tracker.is_routed(prob.customers[index].id) && prob.customers[index] != 0)
                        unrouted.push_back(prob.customers[index]);
                }
                const auto intermediate_customer = std::cbegin(unrouted) + (std::mt19937 {std::random_device{}()}() % unrouted.size());
            #else
                const auto intermediate_customer = std::max_element(std::cbegin(prob.customers), std::cend(prob.customers),
                        [&](Customer const& lhs, Customer const& rhs) {
                    const auto lhs_distance = prob.costs[0][lhs.id];
                    const auto rhs_distance = prob.costs[0][rhs.id];
                    return lhs_distance < rhs_distance && !tracker.is_routed(rhs.id);
                });
            #endif

              if (tracker.is_routed(intermediate_customer->id) || intermediate_customer->id == 0) break;
              current_solution.routes.push_back({iteration, Solution::RouteType {0, 0}});
              auto& constructed = current_solution.routes.back();
              const auto insertion_iterator = constructed.second.insert(std::next(std::cbegin(constructed.second)), intermediate_customer->id);
              tracker.update(constructed.second, insertion_iterator);

              for (;;) {
                  InsertionCandidate best {};
                  for (auto const& current: prob.customers) {
                      if (current.id == 0 || tracker.is_routed(current.id)) continue;
                      InsertionCandidate candidate {};
                      for (auto pos = std::next(std::cbegin(constructed.second)); pos != std::cend(constructed.second); ++pos) {
                          const auto& next = prob.customers[*pos];
                          const auto& prev = prob.customers[*std::prev(pos)];
                          if (!get_insertion_feasiblity(constructed.second, tracker, current, pos)) continue;
                          const auto badness = alpha_evaluation(prev.id, current.id, next.id);
                          if (candidate.id == 0 || badness < candidate.badness) {
                              candidate.id = current.id;
                              candidate.position = pos;
                              candidate.badness = badness;
                              candidate.goodness = beta_evaluation(prev.id, current.id, next.id);
                          }
                      }
                      if (candidate.id == 0) continue;
                      if (best.id == 0 || candidate.goodness > best.goodness) best = candidate;
                  }
                  if (best.id == 0) break;
                  auto iterator_best = constructed.second.insert(best.position, best.id);
                  tracker.update(constructed.second, iterator_best);
              }
            }
            return {solution};
        }
    }
}

