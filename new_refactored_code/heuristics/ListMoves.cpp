#include "ListMoves.h"
#include <iostream>
#include <algorithm>
#include <tuple> 

#include "../core/Objective.h"
#include "LSHelpers.h" 

using namespace std;

namespace LocalSearch {

    struct EdgeDir {
        int a = -1, b = -1; 
    };

    static inline bool edge_exists_fwd(const vector<int>& tour, const vector<int>& pos, int a, int b){
        int K = (int)tour.size();
        int ia = pos[a], ib = pos[b];
        if (ia < 0 || ib < 0) return false;
        return ib == (ia + 1) % K;
    }
    static inline bool edge_exists_rev(const vector<int>& tour, const vector<int>& pos, int a, int b){
        int K = (int)tour.size();
        int ia = pos[a], ib = pos[b];
        if (ia < 0 || ib < 0) return false;
        return ia == (ib + 1) % K;
    }

    struct SavedMove {
        enum Type { INTRA_2OPT, INTER_SWAP } type;
        long long delta;
        EdgeDir rem1, rem2;
        int u = -1, v_out = -1;
        int i_idx = -1, j_idx = -1;
    };

    vector<int> local_search_steepest_with_LM(
        const vector<int>& initial_tour,
        const Instance& I,
        mt19937& rng)
    {
        vector<int> tour = initial_tour;
        LSHelpers h(I.N, I.K, tour);
        const auto& D = I.D;
        const auto& C = I.cost;

        vector<SavedMove> LM; 

        // Evaluate all and fill LM
        auto evaluate_all_and_fill_LM = [&]() -> pair<bool, SavedMove> {
            LM.clear();
            long long best_delta = 0;
            SavedMove best_mv; 
            bool found = false;

            vector<int> nodes_in_tour = tour;
            vector<int> nodes_not_in_tour;
            nodes_not_in_tour.reserve(I.N - I.K);
            for (int x = 0; x < I.N; ++x) if (!h.in_tour[x]) nodes_not_in_tour.push_back(x);

            // 1. INTER SWAP
            for (int u : nodes_in_tour){
                int iu = h.pos[u];
                int pu = h.tour[(iu - 1 + h.K) % h.K];
                int nu = h.tour[(iu + 1) % h.K];
                for (int v_out : nodes_not_in_tour){
                    long long delta = (long long)C[v_out] - C[u] 
                                    + (long long)D[pu][v_out] + D[v_out][nu]
                                    - (long long)D[pu][u] - D[u][nu];
                    if (delta < 0){
                        SavedMove mv;
                        mv.type = SavedMove::INTER_SWAP; mv.delta = delta;
                        mv.rem1 = {pu, u}; mv.rem2 = {u, nu};
                        mv.u = u; mv.v_out = v_out;
                        LM.push_back(mv);
                        if (delta < best_delta){ best_delta = delta; best_mv = mv; found=true; }
                    }
                }
            }

            // 2. INTRA 2-OPT
            for (int i = 0; i < h.K; ++i){
                int u = h.tour[i];
                int v = h.tour[(i+1)%h.K];
                for (int j = i+1; j < h.K; ++j){
                    int x = h.tour[j];
                    int y = h.tour[(j+1)%h.K];
                    if (v == x || u == y) continue;

                    long long delta = (long long)D[u][x] + D[v][y] - D[u][v] - D[x][y];

                    if (delta < 0){
                        SavedMove mv;
                        mv.type = SavedMove::INTRA_2OPT;
                        mv.delta = delta; 
                        mv.rem1 = {u, v}; mv.rem2 = {x, y};
                        mv.i_idx = i; mv.j_idx = j;
                        LM.push_back(mv);
                        if (delta < best_delta){ best_delta = delta; best_mv = mv; found=true; }

                        SavedMove mv_inv = mv;
                        mv_inv.rem1 = {v, u}; mv_inv.rem2 = {y, x};
                        LM.push_back(mv_inv);
                    }
                }
            }

            // Remove best move from LM
            if (found){ 
                for (size_t k=0; k<LM.size(); ++k){
                    if (LM[k].type==best_mv.type && LM[k].delta==best_mv.delta &&
                        LM[k].rem1.a==best_mv.rem1.a && LM[k].u==best_mv.u && 
                        LM[k].v_out==best_mv.v_out){
                        LM.erase(LM.begin()+k);
                        break;
                    }
                }
            }
            return {found, best_mv};
        };

        // Try to reuse LM
        auto try_LM_once = [&]() -> pair<bool, SavedMove> {
            long long best_delta = 0;
            bool found = false;
            SavedMove chosen;
            vector<SavedMove> keep; keep.reserve(LM.size());

            for (auto &mv : LM) {
                if (mv.type == SavedMove::INTER_SWAP) {
                    if (h.pos[mv.u] < 0 || h.in_tour[mv.v_out]) continue; 
                    
                    long long cur_delta = delta_inter_exchange(mv.u, mv.v_out, h, I);
                    if (cur_delta < 0) {
                        if (!found || cur_delta < best_delta) {
                            found = true; best_delta = cur_delta; chosen = mv;
                            chosen.i_idx = -1; chosen.j_idx = -1;
                            chosen.delta = cur_delta;
                        }
                        keep.push_back(mv);
                    }
                    continue;
                }

                // 2-opt
                bool e1_fwd = edge_exists_fwd(h.tour, h.pos, mv.rem1.a, mv.rem1.b);
                bool e1_rev = edge_exists_rev(h.tour, h.pos, mv.rem1.a, mv.rem1.b);
                bool e2_fwd = edge_exists_fwd(h.tour, h.pos, mv.rem2.a, mv.rem2.b);
                bool e2_rev = edge_exists_rev(h.tour, h.pos, mv.rem2.a, mv.rem2.b);

                if ((!e1_fwd && !e1_rev) || (!e2_fwd && !e2_rev)) continue;

                bool same_dir = (e1_fwd && e2_fwd) || (e1_rev && e2_rev);
                if (!same_dir) { keep.push_back(mv); continue; }

                int i = e1_fwd ? h.pos[mv.rem1.a] : h.pos[mv.rem1.b];
                int j = e2_fwd ? h.pos[mv.rem2.a] : h.pos[mv.rem2.b];
                
                if (j < i) std::swap(i, j);
                int v = h.tour[(i + 1) % h.K], x = h.tour[j];
                if (v == x) continue;

                long long cur_delta = delta_intra_exchange_edges_2opt(i, j, h, I);
                if (cur_delta < 0) {
                    if (!found || cur_delta < best_delta) {
                        found = true; best_delta = cur_delta; chosen = mv;
                        chosen.i_idx = i; chosen.j_idx = j;
                        chosen.delta = cur_delta;
                    }
                    keep.push_back(mv);
                }
            }
            LM.swap(keep);

            if (found) {
                for (size_t k = 0; k < LM.size(); ++k) {
                    if (LM[k].type == chosen.type &&
                        LM[k].u == chosen.u && LM[k].v_out == chosen.v_out &&
                        LM[k].rem1.a == chosen.rem1.a && LM[k].rem2.a == chosen.rem2.a) {
                        LM.erase(LM.begin() + k);
                        break;
                    }
                }
                return {true, chosen};
            }
            return {false, {}};
        };

        // Main Loop
        while (true){
            auto [okLM, mvLM] = try_LM_once();
            
            if (okLM){
                if (mvLM.type == SavedMove::INTER_SWAP){
                    apply_inter_exchange(h, mvLM.u, mvLM.v_out);
                } else {
                    apply_intra_exchange_edges_2opt(h, mvLM.i_idx, mvLM.j_idx);
                }
                continue;
            }

            auto [found, best_now] = evaluate_all_and_fill_LM();
            if (!found) break;

            long long actual_delta = (best_now.type == SavedMove::INTER_SWAP)
                ? delta_inter_exchange(best_now.u, best_now.v_out, h, I)
                : delta_intra_exchange_edges_2opt(best_now.i_idx, best_now.j_idx, h, I);

            if (actual_delta < 0) {
                if (best_now.type == SavedMove::INTER_SWAP){
                    apply_inter_exchange(h, best_now.u, best_now.v_out);
                } else {
                    apply_intra_exchange_edges_2opt(h, best_now.i_idx, best_now.j_idx);
                }
            } else {
                continue;
            }
        }
        return tour;
    }
}