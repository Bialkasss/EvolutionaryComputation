#pragma once

#include <vector>
#include <utility> // for std::pair
#include <random>  // for std::mt19937

struct Instance; 

namespace Evolutionary {

    /**
     * Operator 1: Evolutionary algorithm using common subpaths recombination.
     * * @param I The problem instance.
     * @param rng The random number generator.
     * @param time_budget_ms Max execution time in milliseconds.
     * @return A pair containing the best tour found and the number of iterations.
     */
    std::pair<std::vector<int>, int> Evo_op1(const Instance& I, std::mt19937& rng, double time_budget_ms);

    /**
     * Operator 2: Evolutionary algorithm using parent filtering and regret repair.
     * * @param I The problem instance.
     * @param rng The random number generator.
     * @param time_budget_ms Max execution time in milliseconds.
     * @param useLS If true, applies Local Search to the offspring.
     * @return A pair containing the best tour found and the number of iterations.
     */
    std::pair<std::vector<int>, int> Evo_op2(const Instance& I, std::mt19937& rng, double time_budget_ms, bool useLS);

}