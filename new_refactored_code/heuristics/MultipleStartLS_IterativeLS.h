#pragma once
#include <vector>
#include <string>
#include <random>
#include <climits>
#include "../core/Instance.h"

// Struct used to store results of a specific method run
struct MethodResult {
    std::string method; 
    long long best_obj = LLONG_MAX; 
    std::vector<int> best_tour; 
    std::vector<long long> all_objs; 
    std::vector<double> all_times_ms; 
};

namespace Metaheuristics {

    /**
     * @brief Multiple Start Local Search
     * Runs basic LS from many random starts.
     * @return Average runtime per run (useful for setting ILS/LNS budgets).
     */
    double MSLS(const Instance& I, 
                std::mt19937& rng, 
                MethodResult& MR, 
                int outer_runs = 20, 
                int starts_per_run = 200);

    /**
     * @brief Iterated Local Search
     * Applies perturbations (random 2-opt moves) to escape local optima.
     */
    void ILS(const Instance& I,
             std::mt19937& rng,
             MethodResult& MR,
             double time_budget_ms,
             std::vector<int>& ls_runs_per_run,
             int ils_runs = 20,
             int perturb_moves = 3);
}