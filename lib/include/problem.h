#pragma once

#include "vehicle.h"
#include "customer.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <vector>
#include <list>
#include <unordered_map>

namespace vrp {
/// Vehicle Routing Problem representation
class Problem {
public:
    struct VehicleType {
        std::vector<bool> avail_customers;  ///< customers covered by the type
        std::vector<bool> avail_vehicles;   ///< vehicles included in the type
        std::vector<size_t> customers;  ///< customers included in the type
        std::vector<size_t> vehicles;   ///< vehicles included in the type
    };

private:
    std::vector<std::vector<bool>> m_allowed_vehicles;  ///< allowed vehicles
                                                        /// for each customer:
                                                        /// true means allowed,
                                                        /// false otherwise
    std::vector<VehicleType> m_vehicle_types;   ///< vehicle types. size_t value
                                                /// is the vehicle index
    std::vector<std::vector<bool>> m_allowed_types; ///< allowed types
                                                    /// for each customer:
                                                    /// true means allowed,
                                                    /// false otherwise

    template<typename IntegerT>
    std::vector<size_t> to_vector(const std::list<IntegerT>& l) {
        std::vector<size_t> v;
        v.reserve(l.size());
        for (const auto& e : l) v.emplace_back(e);
        return v;
    }

    std::vector<VehicleType> create_vehicle_types() {
        const auto vehicles_size = this->n_vehicles();
        // suitable customers per vehicle
        std::unordered_map<int, std::list<size_t>> suitable_customers = {};
        suitable_customers.reserve(vehicles_size);
        for (const auto& customer : this->customers) {
            int c = customer.id;
            const auto& suitable = customer.suitable_vehicles;
            if (suitable.size() == 0) {
                // if suitable is empty, consider this as "all inclusive"
                // customer: any vehicle can deliver to it
                for (const auto& v : vehicles) {
                    suitable_customers[v.id].emplace_back(c);
                }
            } else {
                // if t is in suitable vehicles, t is allowed for current
                // customer
                for (int v : suitable) {
                    suitable_customers[v].emplace_back(c);
                }
            }
        }
        std::list<int> untyped = {};
        for (size_t i = 0; i < vehicles_size; ++i) {
            untyped.emplace_back(static_cast<int>(i));
        }
        std::list<std::list<int>> vehicle_groups = {};
        while (!untyped.empty()) {
            int v = untyped.back();
            untyped.pop_back();
            const auto& customers = suitable_customers[v];
            bool found_existing_group = false;
            for (auto& g : vehicle_groups) {
                if (g.empty()) continue;
                if (customers == suitable_customers[g.front()]) {
                    g.emplace_back(v);
                    found_existing_group = true;
                }
            }
            if (!found_existing_group) {
                // create new group if there's no existing that matches v
                vehicle_groups.emplace_back(std::list<int>{v});
            }
        }
        const auto customers_size = this->n_customers();
        std::vector<VehicleType> types = {};
        types.reserve(vehicle_groups.size());
        for (const auto& g : vehicle_groups) {
            types.emplace_back(VehicleType{});
            VehicleType& type = types.back();
            const auto& suitable = suitable_customers[g.front()];
            // populate available customers
            type.avail_customers.resize(customers_size, false);
            for (size_t c : suitable)
                type.avail_customers[c] = true;
            // populate available vehicles
            type.avail_vehicles.resize(vehicles_size, false);
            for (size_t v : g)
                type.avail_vehicles[v] = true;
            // populate customers
            type.customers.reserve(suitable.size());
            for (size_t c : suitable)
                type.customers.emplace_back(c);
            // populate vehicles
            type.vehicles.reserve(g.size());
            for (size_t v : g)
                type.vehicles.emplace_back(v);
        }
        return types;
    }

public:
    std::vector<std::vector<double>> costs = {};    ///< cost matrix
    std::vector<Customer> customers = {};   ///< customer list
    std::vector<Vehicle> vehicles = {}; ///< vehicles list
    std::vector<std::vector<int>> times = {};   ///< time matrix
    int max_violated_soft_tw =
        std::numeric_limits<int>::max();    ///< max number of violated
                                            /// soft time windows

    // TODO: should be part of ctor
    void set_up() {
        m_vehicle_types = create_vehicle_types();

        // set allowed types
        const auto customers_size = n_customers();
        const auto types_size = m_vehicle_types.size();
        m_allowed_types.resize(customers_size, {});
        for (size_t c = 0; c < customers_size; ++c) {
            m_allowed_types[c].resize(types_size, false);
        }
        for (size_t c = 0; c < customers_size; ++c) {
            auto& allowed_for_customer = m_allowed_types[c];
            for (size_t t = 0; t < types_size; ++t) {
                const auto& type = m_vehicle_types[t];
                allowed_for_customer[t] = type.avail_customers[c];
            }
        }

        // TODO: get rid of this:
        // set allowed vehicles
        m_allowed_vehicles.resize(customers_size, {});
        for (size_t c = 0; c < customers_size; ++c) {
            const auto& suitable = this->customers[c].suitable_vehicles;
            auto& allowed_for_customer = m_allowed_vehicles[c];
            const auto vehicles_size = n_vehicles();
            if (suitable.size() == 0) {
                // if suitable is empty, consider this as "all inclusive"
                // customer: any vehicle can deliver to it
                allowed_for_customer.resize(vehicles_size, true);
            } else {
                // if t is in suitable vehicles, t is allowed for current
                // customer
                allowed_for_customer.resize(vehicles_size, false);
                for (size_t t = 0; t < vehicles_size; ++t) {
                    allowed_for_customer[t] = (suitable.cend() !=
                        std::find(suitable.cbegin(), suitable.cend(),
                            static_cast<int>(t)));
                }
            }
        }
    }

    inline size_t n_customers() const {
        return this->customers.size();
    }
    inline size_t n_vehicles() const {
        return this->vehicles.size();
    }
    /// Get allowed vehicles for customer. Expected `customer` param is 0-based
    inline const std::vector<bool>& allowed_vehicles(size_t customer) const {
        return m_allowed_vehicles[customer];
    }
    /// Get allowed types for customer. Expected `customer` param is 0-based
    inline const std::vector<bool>& allowed_types(size_t customer) const {
        return m_allowed_types[customer];
    }
    /// Get vehicle types for current problem
    inline const std::vector<VehicleType> vehicle_types() const {
        return m_vehicle_types;
    }
};
}  // vrp
