#include "LocalSearch.h"

#include <vector>
#include <string>
#include <random>       // For mt19937
#include <numeric>      // For iota
#include <algorithm>    // For shuffle, swap, reverse
#include <cmath>        // For fabs
#include <climits>      // For LLONG_MAX
#include <iostream>     // For cerr
#include <tuple>        // For tie, tuple, emplace_back
#include <stdlib.h>     // For exit

#include "../core/Instance.h"
#include "../core/Objective.h" // For Objective::calculate

using namespace std;

// -----------------------------------------------------------------
// ANONYMOUS NAMESPACE (INTERNAL HELPERS)
// All functions and structs here are private to this file.
// -----------------------------------------------------------------
namespace {

/**
 * @brief Helper struct to maintain fast lookups for tour state.
 * Holds references and must be updated after every move.
 */
struct LSHelpers {
    int K, N;
    vector<int>& tour; // Reference to the tour vector
    vector<int> pos;   // pos[node_id] -> index in tour, or -1 if not in tour
    vector<bool> in_tour; // in_tour[node_id] -> true if in tour

    LSHelpers(int n, int k, vector<int>& t) : N(n), K(k), tour(t), pos(n, -1), in_tour(n, false) {
        build();
    }
    
    // (Re)builds the lookup tables from the current tour
    void build() {
        fill(pos.begin(), pos.end(), -1);
        fill(in_tour.begin(), in_tour.end(), false);
        for (int i = 0; i < K; ++i) {
            int node = tour[i];
            pos[node] = i;
            in_tour[node] = true;
        }
    }
};

// --- DELTA CALCULATIONS ---

// Delta: Inter-route calculation (swap node u (in) and v (out))
static long long delta_inter_exchange(int u, int v, const LSHelpers& h, const Instance& I) {
    const auto& D = I.D; const auto& C = I.cost;
    int K = h.K;
    int i = h.pos[u]; // position of u in tour
    
    int prev_u = h.tour[(i - 1 + K) % K];
    int next_u = h.tour[(i + 1) % K];

    long long cost_delta = (long long)C[v] - C[u];
    long long edge_delta = (long long)D[prev_u][v] + D[v][next_u] 
                         - D[prev_u][u] - D[u][next_u];

    return cost_delta + edge_delta;
} 

// Delta: Intra-route (Node exchange) [Swap node at pos i and pos j]
static long long delta_intra_exchange_nodes(int i, int j, const LSHelpers& h, const Instance& I) {
    const auto& D = I.D;
    int K = h.K;
    if (i == j) return 0;
    if (j < i) swap(i, j); // ensure i < j

    int u = h.tour[i];
    int v = h.tour[j];

    int prev_i = h.tour[(i - 1 + K) % K];
    int next_i = h.tour[(i + 1) % K];
    int prev_j = h.tour[(j - 1 + K) % K];
    int next_j = h.tour[(j + 1) % K];

    long long delta = 0;

    if (j == i + 1) { // Adjacent nodes
        // Original: (prev_i) -> u -> v -> (next_j)
        // New:      (prev_i) -> v -> u -> (next_j)
        delta = (long long)D[prev_i][v] + D[v][u] + D[u][next_j]
              - D[prev_i][u] - D[u][v] - D[v][next_j];
    } else if (i == 0 && j == K - 1) { // Adjacent at beginning/end
        // Original: (prev_j) -> v -> u -> (next_i)
        // New:      (prev_j) -> u -> v -> (next_i)
        delta = (long long)D[prev_j][u] + D[u][v] + D[v][next_i]
              - D[prev_j][v] - D[v][u] - D[u][next_i];
    }
    else { // Non-adjacent nodes
        // Original: (prev_i) -> u -> (next_i) ... (prev_j) -> v -> (next_j)
        // New:      (prev_i) -> v -> (next_i) ... (prev_j) -> u -> (next_j)
        delta = (long long)D[prev_i][v] + D[v][next_i] + D[prev_j][u] + D[u][next_j]
              - D[prev_i][u] - D[u][next_i] - D[prev_j][v] - D[v][next_j];
    }
    return delta;
}

// Delta: Intra-route (Edge exchange / 2-Opt)
static long long delta_intra_exchange_edges_2opt(int i, int j, const LSHelpers& h, const Instance& I) {
    const auto& D = I.D;
    int K = h.K;
    if (j < i) swap(i, j); // ensure i < j

    int u = h.tour[i];
    int v = h.tour[(i + 1) % K];
    int x = h.tour[j];
    int y = h.tour[(j + 1) % K];

    // No need to swap adjacent edges
    if (v == x || u == y) return 0;

    // Original: (u) -> (v) ... (x) -> (y)
    // New:      (u) -> (x) ... (v) -> (y)
    long long delta = (long long)D[u][x] + D[v][y] - D[u][v] - D[x][y];
    return delta;
}

// --- MOVE APPLICATIONS ---

// Apply inter-route exchange (swap u (in) and v (out))
static void apply_inter_exchange(LSHelpers& h, int u, int v) {
    int i = h.pos[u];
    
    h.tour[i] = v; // Put v in u's spot
    
    // Update lookups
    h.pos[u] = -1;
    h.in_tour[u] = false;  //Delete u from tour
    h.pos[v] = i;
    h.in_tour[v] = true;   //Add v to tour
}

// Apply intra-route node exchange (swap nodes at pos i and j)
static void apply_intra_exchange_nodes(LSHelpers& h, int i, int j) {
    int u = h.tour[i];
    int v = h.tour[j];

    swap(h.tour[i], h.tour[j]);  //Perform simple swap of nodes
    
    // Update lookups
    h.pos[u] = j;
    h.pos[v] = i;
}

// Apply intra-route edge exchange (2-Opt)
static void apply_intra_exchange_edges_2opt(LSHelpers& h, int i, int j) {
    int K = h.K;
    if (j < i) swap(i, j);

    // This reverses the segment from (i+1) to j, inclusive.
    // E.g., if tour is [0, 1, 2, 3, 4, 5] and i=1, j=4
    // We reverse [2, 3, 4] -> [4, 3, 2]
    // New tour: [0, 1, 4, 3, 2, 5]
       
    // We reverse the segment from index (i+1) to index (j).
    // The edges broken are (i, i+1) and (j, j+1).
    // The new edges are (i, j) and (i+1, j+1).
    reverse(h.tour.begin() + i + 1, h.tour.begin() + j + 1);
    
    // We MUST rebuild the position lookups as they are all invalid
    // in the reversed segment.
    h.build();
}

} // end anonymous namespace

// -----------------------------------------------------------------
// PUBLIC NAMESPACE IMPLEMENTATIONS
// -----------------------------------------------------------------

namespace LocalSearch {

/**
 * @brief Steepest descent local search (Best Improvement).
 * Evaluates all moves and chooses the one with the largest improvement.
 */
vector<int> local_search_steepest(
    const vector<int>& initial_tour, 
    const Instance& I, 
    IntraMoveType intra_mode, 
    mt19937& rng) 
{
    vector<int> tour = initial_tour;
    LSHelpers h(I.N, I.K, tour);
    long long current_obj = Objective::calculate(tour, I);

    vector<int> nodes_in_tour;
    nodes_in_tour.reserve(I.K);
    vector<int> nodes_not_in_tour;
    nodes_not_in_tour.reserve(I.N - I.K);

    while (true) {
        // Rebuild lists of in/out nodes for inter-moves
        nodes_in_tour.clear();
        nodes_not_in_tour.clear();
        for(int i=0; i < I.N; ++i) {
            if (h.in_tour[i]) {
                nodes_in_tour.push_back(i);
            } else {
                nodes_not_in_tour.push_back(i);
            }
        }

        long long best_delta = 0;
        int best_move_type = -1; // 0: inter, 1: intra-node, 2: intra-edge
        int best_u = -1, best_v = -1; // for inter
        int best_i = -1, best_j = -1; // for intra

        // 1. Evaluate all Inter-route moves (node exchange)
        for (int u : nodes_in_tour) {
            for (int v : nodes_not_in_tour) {
                long long delta = delta_inter_exchange(u, v, h, I);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_move_type = 0;
                    best_u = u;
                    best_v = v;
                }
            }
        }

        // 2. Evaluate all Intra-route moves
        if (intra_mode == IntraMoveType::NODE_EXCHANGE) {
            for (int i = 0; i < I.K; ++i) {
                for (int j = i + 1; j < I.K; ++j) {
                    long long delta = delta_intra_exchange_nodes(i, j, h, I);
                    if (delta < best_delta) {
                        best_delta = delta;
                        best_move_type = 1;
                        best_i = i;
                        best_j = j;
                    }
                }
            }
        } else { // EDGE_EXCHANGE_2OPT
            for (int i = 0; i < I.K; ++i) {
                for (int j = i + 1; j < I.K; ++j) { 
                    long long delta = delta_intra_exchange_edges_2opt(i, j, h, I);
                     if (delta < best_delta) {
                        best_delta = delta;
                        best_move_type = 2;
                        best_i = i;
                        best_j = j;
                    }
                }
            }
        }
        
        // 3. Apply best move
        if (best_delta < -1e-9) { // Use tolerance for < 0
            long long old_obj = current_obj; // Use tracked objective
            
            if (best_move_type == 0) {
                apply_inter_exchange(h, best_u, best_v);
            } else if (best_move_type == 1) {
                apply_intra_exchange_nodes(h, best_i, best_j);
            } else { // move_type == 2
                apply_intra_exchange_edges_2opt(h, best_i, best_j);
                // Note: LSHelpers 'h' is now rebuilt
            }
            
            current_obj += best_delta; // Update tracked objective
            
            // Debugging check (can be commented out for speed)
            long long new_obj_calc = Objective::calculate(tour, I); 
            if (std::abs(new_obj_calc - current_obj) > 1) {
                cerr << "FATAL DELTA MISMATCH (STEEP)" 
                     << " Old(tracked): " << old_obj << " Delta: " << best_delta
                     << " New(calc): " << new_obj_calc 
                     << " New(tracked): " << current_obj << endl;
                exit(1);
            }
            current_obj = new_obj_calc; // Resync to avoid drift
        } else {
            break; // Local optimum reached
        }
    }
    return tour;
}

/**
 * @brief Greedy local search (First Improvement).
 * Evaluates moves in random order, accepts the first one with any improvement.
 */
vector<int> local_search_greedy(
    const vector<int>& initial_tour, 
    const Instance& I, 
    IntraMoveType intra_mode, 
    mt19937& rng) 
{
    vector<int> tour = initial_tour;
    LSHelpers h(I.N, I.K, tour);
    long long current_obj = Objective::calculate(tour, I);
    
    // 0: inter {u, v}, 1: intra-node {i, j}, 2: intra-edge {i, j}
    vector<tuple<int, int, int>> moves;

    while (true) {
        bool found_improving = false;

        // 1. Build list of all possible moves
        moves.clear();
        vector<int> nodes_in_tour;
        vector<int> nodes_not_in_tour;
        for (int i = 0; i < I.N; ++i) {
            if (!h.in_tour[i]) {
                nodes_not_in_tour.push_back(i);
            } else {
                nodes_in_tour.push_back(i);
            }
        }

        // Add inter-moves
        for (int u : nodes_in_tour) {
            for (int v : nodes_not_in_tour) {
                moves.emplace_back(0, u, v);
            }
        }
        
        // Add intra-moves
        if (intra_mode == IntraMoveType::NODE_EXCHANGE) {
            for (int i = 0; i < I.K; ++i) {
                for (int j = i + 1; j < I.K; ++j) {
                    moves.emplace_back(1, i, j);
                }
            }
        } else { // EDGE_EXCHANGE_2OPT
            for (int i = 0; i < I.K; ++i) {
                for (int j = i + 1; j < I.K; ++j) {
                    moves.emplace_back(2, i, j);
                }
            }
        }

        // 2. Shuffle moves
        shuffle(moves.begin(), moves.end(), rng);

        // 3. Evaluate moves in random order
        for (const auto& move : moves) {
            long long delta = 0;
            int type, a, b;
            tie(type, a, b) = move;

            if (type == 0) { // inter
                delta = delta_inter_exchange(a, b, h, I);
            } else if (type == 1) { // intra-node
                delta = delta_intra_exchange_nodes(a, b, h, I);
            } else { // intra-edge
                delta = delta_intra_exchange_edges_2opt(a, b, h, I);
            }

            // 4. Apply first improving move
            if (delta < -1e-9) { // Use tolerance
                long long old_obj = current_obj; // Use tracked objective
                
                if (type == 0) {
                    apply_inter_exchange(h, a, b);
                } else if (type == 1) {
                    apply_intra_exchange_nodes(h, a, b);
                } else { // type == 2
                    apply_intra_exchange_edges_2opt(h, a, b);
                    // Note: LSHelpers 'h' is now rebuilt
                }

                current_obj += delta; // Update tracked objective

                // Debugging check (can be commented out for speed)
                long long new_obj_calc = Objective::calculate(tour, I);
                if (std::abs(new_obj_calc - current_obj) > 1) {
                   cerr << "FATAL DELTA MISMATCH (GREEDY)"
                        << " Old(tracked): " << old_obj << " Delta: " << delta
                        << " New(calc): " << new_obj_calc 
                        << " New(tracked): " << current_obj << endl;
                   exit(1);
                }
                current_obj = new_obj_calc; // Resync
                
                found_improving = true;
                break; // Exit inner loop and restart
            }
        }
        
        if (!found_improving) {
            break; // Local optimum reached
        }
    }
    return tour;
}

} // namespace LocalSearch
