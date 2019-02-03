#include <seq_insertion.h>
#include <algorithm>
#include <list>
#include <iostream>
#include <iterator>

#ifndef NDEBUG
    template <typename T>
    std::ostream& operator<<(std::ostream& o, std::list<T> const& list) {
        for (auto it = std::begin(list); it != std::end(list); ++it)
            o << *it << " ";
        return o;
    }

    template <typename T>
    std::ostream& operator<<(std::ostream& o, std::vector<T> const& vector) {
        for (auto const& element: vector)
            o << element << " ";
        return o;
    }
#endif

std::vector<vrp::Solution> sequential_insertion(vrp::Problem& problem) {
    std::vector<std::vector<std::int32_t>> matrix;

    //! @note
    //! randomly-generated experimental data
    //! to ensure the current implementation works as intended

    matrix = {{0, 1, 4,  3,  8  },
              {1, 0, 2,  9,  6  },
              {4, 2, 0,  5,  11 },
              {3, 9, 5,  0,  1  },
              {8, 6, 11, 1,  0  }
             };

    //! @note vertices to be checked
    std::vector<std::uint32_t> vertices;

    const auto alpha_evaluation = [&matrix]
            (std::uint32_t i, std::uint32_t k, std::uint32_t j) {
        static constexpr auto lambda = 1.0;
        return matrix[i][k] + matrix[k][j] + lambda * matrix[i][j];
    };

    const auto beta_evaluation = [&matrix, &alpha_evaluation]
            (std::uint32_t i, std::uint32_t k, std::uint32_t j) {
        static constexpr auto mu = 1.0;
        return mu * matrix[0][k] - alpha_evaluation(i, k, j);
    };

    std::uint32_t total_evaluation;
    auto farthest_customer = std::max_element(std::begin(matrix[0]), std::end(matrix[0]));
    auto distance = std::distance(std::begin(matrix[0]), farthest_customer);
    vertices.push_back(0); vertices.push_back(distance); vertices.push_back(0);

    return {};
};