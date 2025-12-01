#pragma once
#include <vector>
#include <random>
#include "../core/Instance.h"

namespace LNS {

    /**
     * @brief Runs the Large Neighborhood Search metaheuristic.
     * @param T_max_ms Time budget in milliseconds.
     * @param use_ls If true, applies Steepest Local Search (Edge/2-opt) after every Repair.
     */
    std::pair<std::vector<int>, int> run(const Instance& I, double T_max_ms, bool use_ls, std::mt19937& rng);

}