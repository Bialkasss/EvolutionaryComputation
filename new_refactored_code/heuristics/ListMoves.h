#pragma once
#include <vector>
#include <random>
#include "../core/Instance.h"

namespace LocalSearch {

    /**
     * @brief Steepest Local Search using a List of Moves (LM) memory structure
     * to speed up subsequent checks.
     */
    std::vector<int> local_search_steepest_with_LM(
        const std::vector<int>& initial_tour,
        const Instance& I,
        std::mt19937& rng);

}