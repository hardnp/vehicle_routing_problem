#include "savings.h"
#include <algorithm>

namespace vrp {
namespace detail {
std::vector<Solution> savings(const Problem& prob, size_t count) {
	using VehicleIndex = size_t;
	using CustomerIndex = size_t;
	std::vector<std::pair<VehicleIndex, std::vector<CustomerIndex>>> routes;

	for (unsigned int i = 0; i < prob.n_customers; ++i) {
		routes.push_back({ i, {0,i,0} });
	}

	struct S {
		unsigned int i;
		unsigned int j;
		unsigned int sig;
	};

	std::vector<S> save(prob.n_customers * prob.n_customers, {0, 0, 0});

	for (unsigned int i = 0; i < prob.n_customers; ++i) {
		for (unsigned int j = 0; j < prob.n_customers; ++j) {
			save[i*prob.n_customers + j] = {i, j, prob.costs[i][0] + prob.costs[0][j] - prob.costs[i][j] };
		}
	}

	std::sort(save.begin(), save.end(), [](const S& a, const S& b) {return a.sig < b.sig; });
	auto top = save.end();

	while (top > save.begin()) {  // main cycle
		top = save.end() - 1;

		int flag1 = 0; // need 0-i edge
		int flag2 = 0; // need j-0 edge

		for (auto& a : routes) {
			if (a.second[0] == 0 && a.second[1] == top->i) flag1 = 1;
			if (a.second[a.second.size()-1] == 0 && a.second[a.second.size() - 2] == top->j) flag2 = 1;
		}

		if (flag1 && flag2) { // check if route will be feasible

		}

		//
		// put here all the solution staff
		//
	}

	return {};
}
}  // detail
}  // vrp
