#pragma once

#include <vector>
#include <utility>
#include <random>

struct Instance;

namespace Own {

    /**
     * @brief A hybrid evolutionary algorithm that combines two crossover operators.
     * 
     * @param I The problem instance.
     * @param rng The random number generator.
     * @param time_budget_ms Maximum execution time in milliseconds.
     * @param useLS_in_op2 If true, applies Local Search to the offspring of operator 2.
     * @param op1_probability The probability of using crossover operator 1.
     * @return A pair containing the best tour found and the number of iterations.
     */
    std::pair<std::vector<int>, int> hybrid_evolutionary_algorithm(
        const Instance& I, 
        std::mt19937& rng, 
        double time_budget_ms, 
        bool useLS_in_op2, 
        double op1_probability);

}