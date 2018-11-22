#include "problem.h"

namespace vrp {
inline size_t Problem::n_customers() const {
    return this->customers.size();
}

inline size_t Problem::n_vehicles() const {
    return this->vehicles.size();
}
}  // vrp
