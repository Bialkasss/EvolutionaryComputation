#include "OwnMethod.h"

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

#include <numeric>

namespace Own {

    // --- Optimized Hashing ---
    struct TourHasher {
        /**
         * @brief Computes a hash invariant to rotation and reversal.
         * Uses the sum of hashes of undirected edges {u, v}.
         */
        static uint64_t calculate_symmetric_hash(const std::vector<int>& tour) {
            uint64_t total_hash = 0;
            int n = (int)tour.size();
            if (n == 0) return 0;

            for (int i = 0; i < n; ++i) {
                int u = tour[i];
                int v = tour[(i + 1) % n];
                
                // Canonical edge representation: min-max pair
                uint64_t e_min = (u < v) ? u : v;
                uint64_t e_max = (u < v) ? v : u;

                // Simple but effective commutative hash (Zobrist-like or splitmix64)
                // We use a pairing function or just a salted XOR/addition
                uint64_t edge_key = (e_min * 397) ^ (e_max * 743);
                
                // MurmurHash3-style mixer for the edge key
                edge_key ^= edge_key >> 33;
                edge_key *= 0xff51afd7ed558ccdULL;
                edge_key ^= edge_key >> 33;

                total_hash += edge_key; 
            }
            return total_hash;
        }
    };

    struct Solution {
        vector<int> tour;
        long long objective;
        uint64_t hash;

        static Solution create(vector<int>&& tour, const Instance& I) {
            long long obj = Objective::calculate(tour, I);
            // Optimization: Skip get_canonical_tour entirely. 
            // The symmetric hash handles it in O(K).
            uint64_t h = TourHasher::calculate_symmetric_hash(tour);
            return {std::move(tour), obj, h};
        }
    };

    // --- Optimized Repair ---
    static void repair_regret_weighted(vector<int>& tour, const Instance& I, mt19937& rng, double w_reg = 1.0, double w_cost = 1.0) {
        const auto& D = I.D;
        const auto& C = I.cost;
        const auto& nearest_neighbors = I.nearest_neighbors;
        const int CANDIDATE_LIST_SIZE = 20;
        int K = I.K;
        int N = I.N;

        vector<char> used(N, 0);
        vector<int> node_to_pos(N, -1);
        for (size_t i = 0; i < tour.size(); ++i) {
            used[tour[i]] = 1;
            node_to_pos[tour[i]] = (int)i;
        }

        while ((int)tour.size() < K) {
            int m = (int)tour.size();
            if (m == 0) {
                int start_node = uniform_int_distribution<int>(0, N - 1)(rng);
                tour.push_back(start_node);
                used[start_node] = 1;
                node_to_pos[start_node] = 0;
                continue;
            }

            double best_score = -1e300;
            int best_j = -1;
            int best_edge_i = -1;
            long long best1_for_tie = LLONG_MAX;

            for (int j = 0; j < N; ++j) {
                if (used[j]) continue;

                long long cost_b1 = LLONG_MAX;
                long long cost_b2 = LLONG_MAX;
                int pos_b1 = -1;

                // Optimization: Use Candidate List to find best insertion point faster
                int candidates_found = 0;
                for (int neighbor : nearest_neighbors[j]) {
                    int i = node_to_pos[neighbor];
                    if (i != -1) {
                        candidates_found++;
                        // Check two edges adjacent to the neighbor in the current tour
                        int edges_to_check[2] = { i, (i == 0) ? m - 1 : i - 1 };
                        for (int edge_idx : edges_to_check) {
                            int u = tour[edge_idx];
                            int v = tour[(edge_idx + 1) % m];
                            long long total_cost = (long long)D[u][j] + D[j][v] - D[u][v] + C[j];
                            
                            if (total_cost < cost_b1) {
                                cost_b2 = cost_b1; cost_b1 = total_cost; pos_b1 = edge_idx;
                            } else if (total_cost < cost_b2) {
                                cost_b2 = total_cost;
                            }
                        }
                        if (candidates_found >= CANDIDATE_LIST_SIZE) break;
                    }
                }

                // Fallback if no neighbors are in the tour yet
                if (pos_b1 == -1) {
                    for (int i = 0; i < m; ++i) {
                        int u = tour[i], v = tour[(i + 1) % m];
                        long long total_cost = (long long)D[u][j] + D[j][v] - D[u][v] + C[j];
                        if (total_cost < cost_b1) {
                            cost_b2 = cost_b1; cost_b1 = total_cost; pos_b1 = i;
                        } else if (total_cost < cost_b2) {
                            cost_b2 = total_cost;
                        }
                    }
                }

                if (cost_b2 == LLONG_MAX) cost_b2 = cost_b1;
                double score = w_reg * (double)(cost_b2 - cost_b1) - w_cost * (double)cost_b1;

                if (score > best_score) {
                    best_score = score; best_j = j; best_edge_i = pos_b1; best1_for_tie = cost_b1;
                }
            }

            if (best_j != -1) {
                tour.insert(tour.begin() + (best_edge_i + 1), best_j);
                used[best_j] = 1;
                // Only update node_to_pos for shifted elements
                for (size_t i = best_edge_i + 1; i < tour.size(); ++i) node_to_pos[tour[i]] = (int)i;
            } else break;
        }
    }
    

    // --- Start of Refactored Helper Functions ---

    /**
     * @brief Recombination Operator 1: Preserves common subpaths and nodes.
     */
    static vector<int> recombine_common_subpaths(
        const vector<int>& parentA, 
        const vector<int>& parentB, 
        const Instance& I, 
        mt19937& rng
    ) {
        int N = I.N;
        int mA = (int)parentA.size();
        int mB = (int)parentB.size();

        vector<int> posB(N, -1);
        for (int i = 0; i < mB; ++i) posB[parentB[i]] = i;

        vector<vector<int>> subpaths;
        vector<char> used(N, 0);
        vector<char> used_in_A(N, 0);

        for (int i = 0; i < mA; ++i) {
            int u = parentA[i];
            if (posB[u] == -1 || used_in_A[u]) continue;

            vector<int> path;
            path.push_back(u);
            used_in_A[u] = 1;
            int curr_idx_A = i;

            while (true) {
                int next_idx_A = (curr_idx_A + 1) % mA;
                int v = parentA[next_idx_A];
                if (used_in_A[v] || posB[v] == -1) break;

                int idx_u = posB[path.back()];
                int idx_v = posB[v];
                int diff = std::abs(idx_u - idx_v);
                bool is_adjacent_in_B = (diff == 1) || (diff == mB - 1);

                if (is_adjacent_in_B) {
                    path.push_back(v);
                    used_in_A[v] = 1;
                    curr_idx_A = next_idx_A;
                    if (path.size() >= mA) break;
                } else {
                    break;
                }
            }
            subpaths.push_back(std::move(path));
        }

        vector<int> child;
        child.reserve(I.K);

        shuffle(subpaths.begin(), subpaths.end(), rng);
        for (auto &sp : subpaths) {
            if (uniform_int_distribution<int>(0,1)(rng)) reverse(sp.begin(), sp.end());
            for (int v : sp) {
                if (!used[v]) { child.push_back(v); used[v] = 1; }
            }
        }

        vector<int> common_isolated;
        common_isolated.reserve(mA);
        for (int u : parentA) {
            if (!used[u] && posB[u] != -1) {
                common_isolated.push_back(u);
            }
        }
        shuffle(common_isolated.begin(), common_isolated.end(), rng);
        for (int u : common_isolated) {
            child.push_back(u);
            used[u] = 1;
        }

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
    }

    /**
     * @brief Recombination Operator 2: Filters one parent by another, then repairs.
     */
    static vector<int> recombine_filter_repair(
        const vector<int>& start_parent,
        const vector<int>& other_parent,
        const Instance& I,
        mt19937& rng
    ) {
        unordered_set<int> inOther(other_parent.begin(), other_parent.end());
        vector<int> offspring;
        offspring.reserve(start_parent.size());
        for (int v : start_parent) {
            if (inOther.count(v)) {
                offspring.push_back(v);
            }
        }
        repair_regret_weighted(offspring, I, rng);
        return offspring;
    }

    // --- End of Refactored Helper Functions ---


    pair<vector<int>, int> Evo_op1(const Instance& I, mt19937& rng, double time_budget_ms){
        Stopwatch timer;
        vector<Solution> population;
        population.reserve(20);
        unordered_set<uint64_t> population_hashes;
        int iter_count = 0;
        
        while (population.size() < 20) {
            auto S_curr_tour = GreedyHeuristics::random_solution(I, rng, -1);
            S_curr_tour = LocalSearch::local_search_steepest(S_curr_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
            
            auto new_sol = Solution::create(std::move(S_curr_tour), I);

            if (population_hashes.find(new_sol.hash) == population_hashes.end()) {
                population_hashes.insert(new_sol.hash);
                population.push_back(std::move(new_sol));
            }
        }

        while (timer.elapsed_ms() < time_budget_ms) {
            ++iter_count;
            uniform_int_distribution<int> dist(0, (int)population.size() - 1);
            int idxA = dist(rng);        
            int idxB;
            do {idxB = dist(rng);} while (idxA == idxB);
            
            vector<int> offspring_tour = recombine_common_subpaths(population[idxA].tour, population[idxB].tour, I, rng);
            offspring_tour = LocalSearch::local_search_steepest(offspring_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
            
            auto offspring_sol = Solution::create(std::move(offspring_tour), I);

            if (population_hashes.find(offspring_sol.hash) == population_hashes.end()) {
                int worst_idx = 0;
                long long worst_obj = population[0].objective;
                for (int i = 1; i < (int)population.size(); ++i) {
                    if (population[i].objective > worst_obj) {
                        worst_obj = population[i].objective;
                        worst_idx = i;
                    }
                }

                if (offspring_sol.objective < worst_obj) {
                    population_hashes.erase(population[worst_idx].hash);
                    population_hashes.insert(offspring_sol.hash);
                    population[worst_idx] = std::move(offspring_sol);
                }
            }
        }
        int best_idx = 0;
        long long best_obj = population[0].objective;
        for (int i = 1; i < (int)population.size(); ++i) {
            if (population[i].objective < best_obj) { best_obj = population[i].objective; best_idx = i; }
        }
        return { population[best_idx].tour, iter_count };
    }

    pair<vector<int>, int> Evo_op2(const Instance& I, mt19937& rng, double time_budget_ms, bool useLS){
        Stopwatch timer;
        vector<Solution> population;
        population.reserve(20);
        unordered_set<uint64_t> population_hashes;
        int iter_count = 0;
        
        while (population.size() < 20) {
            auto S_curr_tour = GreedyHeuristics::random_solution(I, rng, -1);
            S_curr_tour = LocalSearch::local_search_steepest(S_curr_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
            
            auto new_sol = Solution::create(std::move(S_curr_tour), I);

            if (population_hashes.find(new_sol.hash) == population_hashes.end()) {
                population_hashes.insert(new_sol.hash);
                population.push_back(std::move(new_sol));
            }
        }

        while (timer.elapsed_ms() < time_budget_ms) {
            ++iter_count;
            uniform_int_distribution<int> dist(0, (int)population.size() - 1);
            int idxA = dist(rng);        
            int idxB;
            do {idxB = dist(rng);} while (idxA == idxB);
            
            const auto& pA = population[idxA].tour;
            const auto& pB = population[idxB].tour;

            const vector<int>& start_parent = (uniform_int_distribution<int>(0,1)(rng) == 0) ? pA : pB;
            const vector<int>& other_parent = (&start_parent == &pA) ? pB : pA;

            vector<int> offspring_tour = recombine_filter_repair(start_parent, other_parent, I, rng);

            if (useLS) {
                offspring_tour = LocalSearch::local_search_steepest(offspring_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
            }

            auto offspring_sol = Solution::create(std::move(offspring_tour), I);

            if (population_hashes.find(offspring_sol.hash) == population_hashes.end()) {
                int worst_idx = 0;
                long long worst_obj = population[0].objective;
                for (int i = 1; i < (int)population.size(); ++i) {
                    if (population[i].objective > worst_obj) {
                        worst_obj = population[i].objective;
                        worst_idx = i;
                    }
                }
                if (offspring_sol.objective < worst_obj) {
                    population_hashes.erase(population[worst_idx].hash);
                    population_hashes.insert(offspring_sol.hash);
                    population[worst_idx] = std::move(offspring_sol);
                }
            }
        }
        int best_idx = 0;
        long long best_obj = population[0].objective;
        for (int i = 1; i < (int)population.size(); ++i) {
            if (population[i].objective < best_obj) { best_obj = population[i].objective; best_idx = i; }
        }
        return { population[best_idx].tour, iter_count };
    }

    // New combined function
    std::pair<std::vector<int>, int> hybrid_evolutionary_algorithm(
        const Instance& I, 
        std::mt19937& rng, 
        double time_budget_ms, 
        bool useLS_in_op2, 
        double op1_probability)
    {
        Stopwatch timer;
        vector<Solution> population;
        population.reserve(20);
        unordered_set<uint64_t> population_hashes;
        int iter_count = 0;
        
        // Initial population
        while (population.size() < 20) {
            auto S_curr_tour = GreedyHeuristics::random_solution(I, rng, -1);
            S_curr_tour = LocalSearch::local_search_steepest(S_curr_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
            
            auto new_sol = Solution::create(std::move(S_curr_tour), I);

            if (population_hashes.find(new_sol.hash) == population_hashes.end()) {
                population_hashes.insert(new_sol.hash);
                population.push_back(std::move(new_sol));
            }
        }

        uniform_real_distribution<double> op_dist(0.0, 1.0);

        while (timer.elapsed_ms() < time_budget_ms) {
            ++iter_count;
            
            // Parent selection
            uniform_int_distribution<int> parent_dist(0, (int)population.size() - 1);
            int idxA = parent_dist(rng);        
            int idxB;
            do {idxB = parent_dist(rng);} while (idxA == idxB);
            const auto& pA = population[idxA].tour;
            const auto& pB = population[idxB].tour;

            vector<int> offspring_tour;

            // Crossover with random operator choice
            if (op_dist(rng) < op1_probability) {
                // Operator 1
                offspring_tour = recombine_common_subpaths(pA, pB, I, rng);
                offspring_tour = LocalSearch::local_search_steepest(offspring_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
            } else {
                // Operator 2
                const vector<int>& start_parent = (uniform_int_distribution<int>(0,1)(rng) == 0) ? pA : pB;
                const vector<int>& other_parent = (&start_parent == &pA) ? pB : pA;
                offspring_tour = recombine_filter_repair(start_parent, other_parent, I, rng);
                if (useLS_in_op2) {
                    offspring_tour = LocalSearch::local_search_steepest(offspring_tour, I, LocalSearch::IntraMoveType::EDGE_EXCHANGE_2OPT, rng);
                }
            }

            auto offspring_sol = Solution::create(std::move(offspring_tour), I);

            // Survivor selection (replace worst if better and not duplicate)
            if (population_hashes.find(offspring_sol.hash) == population_hashes.end()) {
                int worst_idx = 0;
                long long worst_obj = population[0].objective;
                for (int i = 1; i < (int)population.size(); ++i) {
                    if (population[i].objective > worst_obj) {
                        worst_obj = population[i].objective;
                        worst_idx = i;
                    }
                }

                if (offspring_sol.objective < worst_obj) {
                    population_hashes.erase(population[worst_idx].hash);
                    population_hashes.insert(offspring_sol.hash);
                    population[worst_idx] = std::move(offspring_sol);
                }
            }
        }

        // Return best
        int best_idx = 0;
        long long best_obj = population[0].objective;
        for (int i = 1; i < (int)population.size(); ++i) {
            if (population[i].objective < best_obj) {
                best_obj = population[i].objective;
                best_idx = i;
            }
        }
        return { population[best_idx].tour, iter_count };
    }
}
