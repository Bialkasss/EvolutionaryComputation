#include "HybridEvolutionary.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <climits>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <random>
#include <cstdint>
#include <utility> 

#include "../core/Instance.h"
#include "../core/Objective.h"
#include "../utils/Timer.h"
#include "LocalSearch.h"  
#include "Greedy.h" 

using namespace std;

namespace Evolutionary {

    /**
     * @brief Repair operator similar to Large Neighborhood Search (LNS) repair.
     * * It greedily inserts missing nodes into the tour to reach size K based on a 
     * weighted score of Regret and Cost.
     * * @param tour The partial tour to repair.
     * @param I The problem instance.
     * @param rng Random number generator.
     * @param w_reg Weight for the Regret value (default 1.0).
     * @param w_cost Weight for the Cost value (default 1.0).
     */
    static void repair_regret_weighted(vector<int>& tour, const Instance& I, mt19937& rng, double w_reg = 1.0, double w_cost = 1.0) {
        const auto& D = I.D;
        const auto& C = I.cost;
        int K = I.K;
        int N = I.N;

        // 1. Identify used nodes based on the PARTIAL tour passed in
        vector<char> used(N, 0);
        for (int x : tour) used[x] = 1;

        // 2. Loop until size is K
        while ((int)tour.size() < K) {
            int m = (int)tour.size();

            double best_score = -1e300;
            long long best1_for_tie = LLONG_MAX;
            int best_j = -1;
            int best_edge_i = -1;

            for (int j = 0; j < N; ++j) if (!used[j]) {

                long long cost_b1 = LLONG_MAX;
                long long cost_b2 = LLONG_MAX;
                int pos_b1 = -1;

                for (int i = 0; i < m; ++i) {
                    int u = tour[i];
                    int v = tour[(i+1)%m];

                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    long long total_cost = delta + C[j];

                    if (total_cost < cost_b1) {
                        cost_b2 = cost_b1;
                        cost_b1 = total_cost;
                        pos_b1 = i;
                    } else if (total_cost < cost_b2) {
                        cost_b2 = total_cost;
                    }
                }

                if (cost_b2 == LLONG_MAX) cost_b2 = cost_b1;

                long long regret = cost_b2 - cost_b1;

                // Weighted Score
                double score = w_reg * (double)regret - w_cost * (double)cost_b1;

                if (score > best_score) {
                    best_score = score;
                    best_j = j;
                    best_edge_i = pos_b1;
                    best1_for_tie = cost_b1;
                } else if (std::abs(score - best_score) < 1e-12) {
                    if (cost_b1 < best1_for_tie ||
                        (cost_b1 == best1_for_tie && uniform_int_distribution<int>(0,1)(rng))) {
                        best_j = j;
                        best_edge_i = pos_b1;
                        best1_for_tie = cost_b1;
                    }
                }
            }

            if (best_j != -1) {
                tour.insert(tour.begin() + (best_edge_i + 1), best_j);
                used[best_j] = 1;
            } else {
                break;
            }
        }
    }


    /**
     * @brief Operator 1: Recombination via Common Subpaths + Random Completion.
     * * 1. Preserves common edges (undirected) from parents.
     * 2. Preserves common isolated nodes.
     * 3. Fills remainder randomly.
     * 4. Applies Local Search to every offspring.
     */
    pair<vector<int>, int> Evo_op1(const Instance& I, mt19937& rng, double time_budget_ms){
        Stopwatch timer;

        // Initial population of 20 Local Searches from random initialization (ensuring no duplicate exists)
        vector<pair<vector<int>, long long>> population;
        population.reserve(20);
        int iter_count = 0;
        while ((int)population.size() < 20) {
            auto S_curr = GreedyHeuristics::random_solution(I, rng, -1);
            S_curr = LocalSearch::local_search_steepest(S_curr, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
            long long obj = Objective::calculate(S_curr, I);
            bool duplicate = false;
            for (const auto &p : population) {
                if (p.first == S_curr) { duplicate = true; break; }
            }
            if (!duplicate) {
                population.emplace_back(std::move(S_curr), obj);
            }
        }

        while (timer.elapsed_ms() < time_budget_ms) {
            ++iter_count;
            // Take 2 parents from uniform distribution
            uniform_int_distribution<int> dist(0, (int)population.size() -1);
            int idxA = dist(rng);        
            int idxB;
            do {idxB = dist(rng);} while (idxA == idxB);
            auto &parentA = population[idxA];
            auto &parentB = population[idxB];

            //// Create offspring
            // Operator 1
            // Helper: build child preserving common edges first, then common nodes (preserve parentA order)
            auto build_child_preserve_common = [&](const vector<int>& A, const vector<int>& B) {

                // Initialize the child
                int N = I.N;
                int mA = (int)A.size();
                int mB = (int)B.size();

                //Mapping values in B to indices for O(1) adjacency checks
                vector<int> posB(N, -1);
                for (int i = 0; i < mB; ++i) posB[B[i]] = i;

                vector<vector<int>> subpaths;
                vector<char> used(N, 0);       // Tracks nodes in the final child
                vector<char> used_in_A(N, 0);  // Tracks nodes processed in A loop to skip overlaps


                for (int i = 0; i < mA; ++i) {
                    int u = A[i];

                    // Skip if u is not in B or already part of a processed subpath
                    if (posB[u] == -1 || used_in_A[u]) continue;

                    vector<int> path;
                    path.push_back(u);
                    used_in_A[u] = 1;
                    int curr_idx_A = i;

                    while (true) {
                        // Check the next node in A
                        int next_idx_A = (curr_idx_A + 1) % mA;
                        int v = A[next_idx_A];

                        // Stop if v is already used or not in B
                        if (used_in_A[v] || posB[v] == -1) break;

                        // Check if u and v are adjacent in B (undirected)
                        int idx_u = posB[path.back()];
                        int idx_v = posB[v];
                        
                        // Calculate distance in B (handling wrap-around)
                        int diff = std::abs(idx_u - idx_v);
                        bool is_adjacent_in_B = (diff == 1) || (diff == mB - 1);

                        if (is_adjacent_in_B) {
                            // It's a common edge! Extend path.
                            path.push_back(v);
                            used_in_A[v] = 1;
                            curr_idx_A = next_idx_A; // Advance
                            
                            // Safety: stop if we consumed the whole tour
                            if (path.size() >= mA) break;
                        } else {
                            // Edge broken
                            break;
                        }
                    }
                    subpaths.push_back(std::move(path));
                }
                vector<int> child;
                child.reserve(I.K);

                // Shuffle subpaths order and randomly flip orientation when concatenating
                shuffle(subpaths.begin(), subpaths.end(), rng);
                for (auto &sp : subpaths) {
                    if (uniform_int_distribution<int>(0,1)(rng)) reverse(sp.begin(), sp.end());
                    for (int v : sp) {
                        if (!used[v]) { child.push_back(v); used[v] = 1; }
                    }
                }

                // Nodes present in both A and B but were not part of a common edge
                vector<int> common_isolated;
                common_isolated.reserve(mA);
                for (int u : A) {
                    if (!used[u] && posB[u] != -1) {
                        common_isolated.push_back(u);
                    }
                }
                // Shuffle these isolated nodes and add them
                shuffle(common_isolated.begin(), common_isolated.end(), rng);
                for (int u : common_isolated) {
                    child.push_back(u);
                    used[u] = 1;
                }

                // Fill remaining positions with random unused nodes until size K
                if ((int)child.size() < I.K) {
                    vector<int> pool;
                    pool.reserve(N);
                    for (int x = 0; x < N; ++x) if (!used[x]) pool.push_back(x);
                    shuffle(pool.begin(), pool.end(), rng);
                    int need = I.K - (int)child.size();
                    for (int t = 0; t < need && t < (int)pool.size(); ++t) {
                        child.push_back(pool[t]);
                        used[pool[t]] = 1;
                    }
                }

                if ((int)child.size() > I.K) child.resize(I.K);
                return child;
            };

            // Create one child. Apply LS to find optimum locally.
            vector<int> offspring = build_child_preserve_common(parentA.first, parentB.first);
            offspring = LocalSearch::local_search_steepest(offspring, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
            

            // Check if offspring is not a duplicate (compare full tour)
            bool is_duplicate = false;
            for (const auto& p : population) {
                if (p.first == offspring) { is_duplicate = true; break; }
            }

            // Find worst solution in population
            if (!is_duplicate) {
                int worst_idx = 0;
                long long worst_obj = population[0].second;
                for (int i = 1; i < (int)population.size(); ++i) {
                if (population[i].second > worst_obj) {
                    worst_obj = population[i].second;
                    worst_idx = i;
                }
                }

                // If offspring is better than worst, replace it
                long long offspring_obj = Objective::calculate(offspring, I);
                if (offspring_obj < worst_obj) {
                population[worst_idx] = {std::move(offspring), offspring_obj};
                }
            }
        }
        // take best from population as a return
        int best_idx = 0;
        long long best_obj = population[0].second;
        for (int i = 1; i < (int)population.size(); ++i) {
            if (population[i].second < best_obj) { best_obj = population[i].second; best_idx = i; }
        }
        return { population[best_idx].first, iter_count };
        
    }

    /**
     * @brief Operator 2: Recombination via Filter + Repair.
     * * 1. Start with Parent A.
     * 2. Remove all nodes NOT present in Parent B. (Preserves A's order/direction).
     * 3. Repair using Regret Heuristic.
     * 4. Optional Local Search.
     */
    pair<vector<int>, int> Evo_op2(const Instance& I, mt19937& rng, double time_budget_ms, bool useLS){
        Stopwatch timer;

        // Initial population of 20 Local Searches from random initialization (ensuring no duplicate exists)
        vector<pair<vector<int>, long long>> population;
        population.reserve(20);
        int iter_count = 0;
        while ((int)population.size() < 20) {
            auto S_curr = GreedyHeuristics::random_solution(I, rng, -1);
            S_curr = LocalSearch::local_search_steepest(S_curr, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
            long long obj = Objective::calculate(S_curr, I);
            bool duplicate = false;
            for (const auto &p : population) {
                if (p.first == S_curr) { duplicate = true; break; }
            }
            if (!duplicate) {
                population.emplace_back(std::move(S_curr), obj);
            }
        }

        while (timer.elapsed_ms() < time_budget_ms) {
            ++iter_count;
            // Take 2 parents from uniform distribution
            uniform_int_distribution<int> dist(0, (int)population.size() -1);
            int idxA = dist(rng);        
            int idxB;
            do {idxB = dist(rng);} while (idxA == idxB);
            auto &parentA = population[idxA];
            auto &parentB = population[idxB];

            // Operator 2: pick one parent as start, remove nodes not in the other, repair with regret heuristic
            const vector<int>& start_parent = (uniform_int_distribution<int>(0,1)(rng) == 0) ? parentA.first : parentB.first;
            const vector<int>& other_parent = (&start_parent == &parentA.first) ? parentB.first : parentA.first;

            // Filter start_parent to keep only nodes present in other_parent (preserve order and direction)
            unordered_set<int> inOther(other_parent.begin(), other_parent.end());
            vector<int> offspring;
            offspring.reserve(start_parent.size());
            for (int v : start_parent) {
                if (inOther.find(v) != inOther.end()) offspring.push_back(v);
            }

            // Repair using regret-weighted heuristic to reach size K
            repair_regret_weighted(offspring, I, rng);

            // Optionally apply local search to offspring
            if (useLS) {
                offspring = LocalSearch::local_search_steepest(offspring, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
            }

            long long offspring_obj = Objective::calculate(offspring, I);

            // Check duplicates (full tour equality)
            bool is_duplicate = false;
            for (const auto &p : population) {
                if (p.first == offspring) { is_duplicate = true; break; }
            }

            if (!is_duplicate) {
                int worst_idx = 0;
                long long worst_obj = population[0].second;
                for (int i = 1; i < (int)population.size(); ++i) {
                    if (population[i].second > worst_obj) { worst_obj = population[i].second; worst_idx = i; }
                }
                if (offspring_obj < worst_obj) {
                    population[worst_idx] = { std::move(offspring), offspring_obj };
                }
            }
        }

        // take best from population as a return
        int best_idx = 0;
        long long best_obj = population[0].second;
        for (int i = 1; i < (int)population.size(); ++i) {
            if (population[i].second < best_obj) { best_obj = population[i].second; best_idx = i; }
        }
        return { population[best_idx].first, iter_count };


    }
    

}