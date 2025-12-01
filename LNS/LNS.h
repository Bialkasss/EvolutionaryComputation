#pragma once
#include <vector>
#include <random>
#include "../core/Instance.h"

namespace LNS {

    /**
     * @brief Runs the Large Neighborhood Search metaheuristic.
     * * @param I The problem instance.
     * @param T_max_ms Time budget in milliseconds.
     * @param use_ls If true, applies Steepest Local Search after Repair.
     * @param rng Random number generator.
     * @return The best solution found.
     */
    std::vector<int> run(const Instance& I, double T_max_ms, bool use_ls, std::mt19937& rng);

}