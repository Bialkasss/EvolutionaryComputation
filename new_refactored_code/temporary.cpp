#include <bits/stdc++.h>
using namespace std;
#include <iomanip>
#include <chrono>
#include <filesystem>

// --------------------- Timer ---------------------
using hires_clock = std::chrono::high_resolution_clock;
using duration_ms = std::chrono::duration<double, std::milli>;

struct Stopwatch {
    hires_clock::time_point start;
    Stopwatch() : start(hires_clock::now()) {}
    void reset() { start = hires_clock::now(); }
    double elapsed_ms() const {
        return duration_ms(hires_clock::now() - start).count();
    }
};

// Simple in-terminal progress bar
void print_progress(double progress, const string& label) {
    int bar_width = 40; // width of the bar
    // Ensure progress is capped at 1.0
    if (progress > 1.0) progress = 1.0;
    if (progress < 0.0) progress = 0.0;
    int pos = (int)(bar_width * progress);
    
    cerr << "  [";
    for (int i = 0; i < bar_width; ++i) {
        if (i < pos) cerr << "=";
        else if (i == pos) cerr << ">";
        else cerr << " ";
    }
    cerr << "] " << (int)(progress * 100.0 + 0.5) << "% "
         << label << "                \r"; // Add padding and \r
    cerr.flush();
}

// --------------------- Data structures ---------------------
struct Point { int x{}, y{}; };

struct Instance {
    string name;
    vector<Point> pts;         // for SVG only; heuristics DO NOT use pts
    vector<int>    cost;       // node cost
    vector<vector<int>> D;     // integer-rounded distances
    int N = 0;
    int K = 0;                 // ceil(N/2)
};

static inline int iround(double v) {
    return (int) llround(v);
}

// Parsing  basically it reads the csv file line by line
// and splits each line into tokens based semicolons or whitespace
// then it expects at least 3 tokens per line: x, y, cost
// it returns the list of strings -> tokens that is essentially point coordinates and cost
static vector<string> split_tokens(const string& line) {
    vector<string> out;
    string token;
    for (size_t i = 0; i < line.size(); ++i) {
        char c = line[i];
        if (c == ';' || isspace((unsigned char)c)) {
            if (!token.empty()) { out.push_back(token); token.clear(); }
        } else {
            token.push_back(c);
        }
    }
    if (!token.empty()) out.push_back(token);
    return out;
}


// it reads the instance from a file -> so tspa or tspb basiccally
// 
bool read_instance(const string& path, Instance& inst) {
    ifstream in(path); //open file
    if (!in) { // check if file opened successfully
        cerr << "Cannot open: " << path << "\n";
        return false;
    }

    inst.name = filesystem::path(path).stem().string(); // get file name without extension or foldernames
    //clearing the inst object in case it was 'foll' so we will not get overwritting
    inst.pts.clear(); 
    inst.cost.clear(); 

    // read points as lines
    string line;
    while (getline(in, line)) {
        string s = line;
        // triming
        auto start = s.find_first_not_of(" \t\r\n"); // skip leading whitespace, tabs, newlines and returns the index of the first occurrence
        if (start == string::npos) continue; //skip empty line
        s = s.substr(start); // remove leading whitespace
        if (s.rfind("#", 0) == 0) continue;  // skip comment lines
        if (s.rfind("//", 0) == 0) continue; // skip comment lines

        auto toks = split_tokens(s); // split line into tokens
        if (toks.empty()) continue; // skip empty line
        // we need at least 3 tokens per line: x, y, cost if some dont have it we skip them
        if (toks.size() < 3) { 
            cerr << "Skipping malformed line: " << line << "\n";
            continue;
        }

        //we convert the tokens to integers and store them in the instance
        Point p{ stoi(toks[0]), stoi(toks[1]) }; //create a point with x and y coordinates
        int c = stoi(toks[2]); // node cost
        inst.pts.push_back(p); //list of points(coordinates)
        inst.cost.push_back(c); //list of costs
    }

    inst.N = (int)inst.pts.size(); // number of nodes -- how many points have we actually read
    if (inst.N < 3) {
        cerr << "Instance must have at least 3 nodes.\n";
        return false;
    }
    inst.K = (inst.N + 1) / 2; // we choose exactly 50% of nodes - or ceil(N/2) if N is odd


    // compute DISTANCE matrix
    inst.D.assign(inst.N, vector<int>(inst.N, 0)); // initialize distance matrix with zeros nxn size n is number of nodes
    for (int i = 0; i < inst.N; ++i) {
        for (int j = i+1; j < inst.N; ++j) {
            long dx = (long)inst.pts[i].x - inst.pts[j].x;
            long dy = (long)inst.pts[i].y - inst.pts[j].y;
            int d = iround(sqrt((double)dx*dx + (double)dy*dy)); // Euclidean distance rounded to nearest integer
            inst.D[i][j] = inst.D[j][i] = d; //symmetric matrix 
        }
    }
    return true;
}

// --------------------- Objective / checking ---------------------
// tour is a list of node indices that were selected
// D is precumputed distance matrix
// we sum distances along the cycle - tour 
long long tour_edge_sum(const vector<int>& tour, const vector<vector<int>>& D) {
    long long s = 0;
    int m = (int)tour.size();
    //if (m == 0) return 0;
    for (int i = 0; i < m; ++i) {
        int u = tour[i];
        int v = tour[(i+1)%m]; //this wraps last node to the first node
        s += D[u][v];
    }
    return s;
}

// sum of costs of selected nodes
long long tour_cost_sum(const vector<int>& tour, const vector<int>& C) {
    long long s = 0;
    for (int v : tour) s += C[v]; //we just summ up costs of the nodes that are in the tour
    return s;
}

//the actual objective that we want to minimize ---> it is a sum of distances and costs
long long objective(const vector<int>& tour, const Instance& I) {
    return tour_edge_sum(tour, I.D) + tour_cost_sum(tour, I.cost);
}


//this is mostly for my sanity just to check that the solution is valid
// it checks that the tour has exactly K nodes, all indices are in range and no node is repeated
bool check_solution(const vector<int>& tour, const Instance& I, string* why=nullptr) {
    int K = I.K;
    if ((int)tour.size() != K) { if (why) *why = "wrong size"; return false; }

    vector<int> seen(I.N, 0);
    
    for (int v : tour) {

        if (v < 0 || v >= I.N) { if (why) *why = "index out of range"; return false; }
        if (seen[v]) { if (why) *why = "repeated node"; return false; }
        seen[v] = 1;
    }
    // edges exist by definition in D
    return true;
}

// --------------------- Helpers ---------------------
template<class T>

// return index of minimum value in vals; random tie break
// rng is a random number generator
// vals is candidate scores - so basically their objective values that we want to minimize further on
int argmin_random_tie(const vector<T>& vals, mt19937& rng) {
    T best = vals[0];
    vector<int> idx{0};
    for (int i = 1; i < (int)vals.size(); ++i) {
        if (vals[i] < best) { best = vals[i]; idx.assign(1, i); }
        else if (vals[i] == best) idx.push_back(i);
    }
    uniform_int_distribution<int> dist(0, (int)idx.size()-1);
    return idx[dist(rng)];
}

// insert val into vec at position pos
template<typename T>
void insert_at(vector<T>& v, int pos, const T& val) {
    v.insert(v.begin()+pos, val);
}

// ---------- LM (list of improving moves) support ----------
struct EdgeDir {
    int a=-1, b=-1; // directed edge a->b
};

static inline bool edge_exists_fwd(const vector<int>& tour, const vector<int>& pos, int a, int b){
    int K = (int)tour.size();
    int ia = pos[a], ib = pos[b];
    if (ia < 0 || ib < 0) return false;
    return ib == (ia + 1) % K;
}
static inline bool edge_exists_rev(const vector<int>& tour, const vector<int>& pos, int a, int b){
    // does b->a exist?
    int K = (int)tour.size();
    int ia = pos[a], ib = pos[b];
    if (ia < 0 || ib < 0) return false;
    return ia == (ib + 1) % K;
}
static inline int idx_of(const vector<int>& pos, int node){ return pos[node]; }

// Saved move from previous iteration
struct SavedMove {
    enum Type { INTRA_2OPT, INTER_SWAP } type;
    long long delta;
    EdgeDir rem1, rem2;
    int u=-1, v_out=-1;

    // only valid for freshly evaluated 2-opt (not for LM reuse)
    int i_idx=-1, j_idx=-1;
};




// // ------------------- Delta Calculations --------------------------------------
// // Intra-route two nodes exchange (swap positions of the nodes)
long long delta_nodes_exchange(const vector<int>& tour, int i, int j, 
                               const vector<vector<int>>& D) {
    int m = (int)tour.size();
    if (i == j) return 0;
    if (i > j) swap(i, j);  // ensure i < j for better logic
    
    // Special case: adjacent nodes -> (less computation)
    if (j == i + 1) {
        // Before: ... - tour[i-1] - tour[i] - tour[j] - tour[j+1] - ...
        // After:  ... - tour[i-1] - tour[j] - tour[i] - tour[j+1] - ...
        int prev = tour[(i - 1 + m) % m];
        int next = tour[(j + 1) % m];
        int ni = tour[i];
        int nj = tour[j];
        
        long long old_cost = (long long)D[prev][ni] + D[ni][nj] + D[nj][next];
        long long new_cost = (long long)D[prev][nj] + D[nj][ni] + D[ni][next];
        return new_cost - old_cost;
    }
    
    // General case: non-adjacent nodes
    // Before: ... - tour[i-1] - tour[i] - tour[i+1] - ... - tour[j-1] - tour[j] - tour[j+1] - ...
    // After:  ... - tour[i-1] - tour[j] - tour[i+1] - ... - tour[j-1] - tour[i] - tour[j+1] - ...

    int prev_i = tour[(i - 1 + m) % m];
    int next_i = tour[(i + 1) % m];
    int prev_j = tour[(j - 1 + m) % m];
    int next_j = tour[(j + 1) % m];
    int ni = tour[i];
    int nj = tour[j];
    
    long long old_cost = (long long)D[prev_i][ni] + D[ni][next_i] + 
                         D[prev_j][nj] + D[nj][next_j];
    long long new_cost = (long long)D[prev_i][nj] + D[nj][next_i] + 
                         D[prev_j][ni] + D[ni][next_j];
    return new_cost - old_cost;
} 

//Intra-route 2 edges exchange (reverse segment between edges)
long long delta_edges_exchange(const vector<int>& tour, int i, int j, 
                               const vector<vector<int>>& D) {
    int m = (int)tour.size();
    if (i == j) return 0;
    
    // Ensure i < j and valid range
    if (i > j) swap(i, j);
    if (j - i == 1) return 0;  // adjacent edges, no change
    
    // Before: tour[i] - tour[i+1] - ... - tour[j] - tour[j+1]
    // After:  tour[i] - tour[j] - ... - tour[i+1] - tour[j+1]
    int ni = tour[i];
    int ni_next = tour[(i + 1) % m];
    int nj = tour[j];
    int nj_next = tour[(j + 1) % m];
    
    long long old_cost = (long long)D[ni][ni_next] + D[nj][nj_next];
    long long new_cost = (long long)D[ni][nj] + D[ni_next][nj_next];
    return new_cost - old_cost;
}

//Inter-route (exchange 1 node in tour with 1 outside)
long long delta_inter_exchange(const vector<int>& tour, int pos, 
                               int node_out, const Instance& I) {
    const auto& D = I.D;
    const auto& C = I.cost;
    int m = (int)tour.size();
    
    int node_in = tour[pos];
    int prev = tour[(pos - 1 + m) % m];
    int next = tour[(pos + 1) % m];
    
    // Edge cost change
    long long old_edges = (long long)D[prev][node_in] + D[node_in][next];
    long long new_edges = (long long)D[prev][node_out] + D[node_out][next];
    
    // Node cost change
    long long cost_change = C[node_out] - C[node_in];
    
    return (new_edges - old_edges) + cost_change;
}

// ------------------- Apply changes in cycle -------------------
// Apply two-nodes exchange
void apply_nodes_exchange(vector<int>& tour, int i, int j) {
    swap(tour[i], tour[j]);
}

// Apply two-edges exchange (reverse segment)
void apply_edges_exchange(vector<int>& tour, int i, int j) {
    int m = (int)tour.size();
    // Reverse segment from (i+1) to j inclusive
    reverse(tour.begin() + i + 1, tour.begin() + j + 1);
}

// Apply inter-route exchange
void apply_inter_exchange(vector<int>& tour, int pos, int node_out) {
    tour[pos] = node_out;
}

//  Move structure
struct Move {
    enum Type { INTRA_NODES, INTRA_EDGES, INTER } type;

    int i, j;           // indices for intra moves, or pos + node_out for inter
    long long delta;
    
    bool operator<(const Move& other) const {
        return delta < other.delta;  // check if its smaller (minimum)
    }
};

// Perturbations used in ILS
// basically we apply a few random 2-opt moves to the tour
void perturb_tour_2opt(vector<int>& tour, mt19937& rng, int num_moves) {
    int K = (int)tour.size();
    if (K < 4) return;

    uniform_int_distribution<int> dist(0, K - 1);

    int tries = 0;
    for (int m = 0; m < num_moves && tries < num_moves * 10; ) {
        int i = dist(rng);
        int j = dist(rng);
        ++tries;
        if (i == j) continue;
        if (i > j) swap(i, j);

        // avoid trivial 2-opt (adjacent edges or whole cycle)
        if (j - i <= 1 || (i == 0 && j == K - 1)) continue;

        apply_edges_exchange(tour, i, j);   // your earlier function working on raw tour
        ++m;
    }
}



// --------------------- Heuristics ---------------------
// Random (200 total; not per start). If start_anchor >=0, force include it.
vector<int> random_solution(const Instance& I, mt19937& rng, int start_anchor = -1) {
    
    vector<int> nodes(I.N); //vector of all nodes
    iota(nodes.begin(), nodes.end(), 0); // fill with 0,1,2,...,N-1

    // if we have a start anchor we make sure it is included in the solution - this is just for my experiments :)
    if (start_anchor >= 0) {
        swap(nodes[0], nodes[start_anchor]);
        // choose K-1 from remaining and include anchor
        shuffle(nodes.begin()+1, nodes.end(), rng); // shuffle all but the anchor

        vector<int> sel; 
        sel.reserve(I.K); //space for K nodes that we need for the cycle creation
        sel.push_back(nodes[0]); // include anchor
        for (int i = 1; (int)sel.size() < I.K; ++i) sel.push_back(nodes[i]); // fill up to K nodes
        shuffle(sel.begin(), sel.end(), rng); // shuffle the selected nodes to get a random order but always including the anchor
        return sel;
    } else {
        shuffle(nodes.begin(), nodes.end(), rng); // shuffle all nodes
        vector<int> sel(nodes.begin(), nodes.begin()+I.K); // take first K nodes
        shuffle(sel.begin(), sel.end(), rng); // shuffle the selected nodes to get a random order
        return sel;
    }
}



// --------------------- GREEDY REGRET HEURISTICS - GREEDY CYCLE ---------------------


// Greedy weighted: maximize (w_reg * regret - w_cost * best1)
// By default w_reg = w_cost = 1 we change the weights further on to 2,1 and 1,2 combinations in our experiments
vector<int> greed_cycle_regret2_weighted(const Instance& I, int start, mt19937& rng,
                                   double w_reg = 1.0, double w_cost = 1.0) {

    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N;
    vector<char> used(N, 0);
    vector<int> cyc; cyc.reserve(K);
    cyc.push_back(start); used[start] = 1;

    while ((int)cyc.size() < K) {
        int m = (int)cyc.size();

        double best_score = -1e300;         
        long long best1_for_tie = LLONG_MAX;  
        int best_j = -1, best_edge_i = -1;

        for (int j = 0; j < N; ++j) if (!used[j]) {
            vector<long long> costs;
            vector<int> edges;
            if (m == 1) {
                int s = cyc[0];
                costs.push_back(2LL * D[s][j] + C[j]);
                edges.push_back(0);
            } else {
                costs.reserve(m);
                edges.reserve(m);
                for (int i = 0; i < m; ++i) {
                    int u = cyc[i], v = cyc[(i+1)%m];
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    costs.push_back(delta + C[j]);
                    edges.push_back(i);
                }
            }

            // find best1 and best2
            int b1 = 0, b2 = (int)costs.size()>1 ? 1 : 0;
            if (b2 && costs[b2] < costs[b1]) swap(b1, b2);
            for (int t = 2; t < (int)costs.size(); ++t) {
                if (costs[t] < costs[b1]) { b2 = b1; b1 = t; }
                else if (b2 == b1 || costs[t] < costs[b2]) { b2 = t; }
            }
            long long best1 = costs[b1];
            long long best2 = costs[b2];
            long long regret = best2 - best1;

            double score = w_reg * (double)regret - w_cost * (double)best1;

            if (score > best_score) {
                best_score = score;
                best_j = j;
                best_edge_i = edges[b1];
                best1_for_tie = best1;
            } else if (fabs(score - best_score) < 1e-12) {
                // tie-break by smaller best1, then random
                if (best1 < best1_for_tie ||
                    (best1 == best1_for_tie && uniform_int_distribution<int>(0,1)(rng))) {
                    best_j = j;
                    best_edge_i = edges[b1];
                    best1_for_tie = best1;
                }
            }
        }

        cyc.insert(cyc.begin() + (best_edge_i + 1), best_j);
        used[best_j] = 1;
    }
    return cyc;
}


// ---------------------------------- LOCAL SEARCH -------------------------------------------------
// ----- LS HELPERS -----
struct LSHelpers {
    int K, N;
    vector<int>& tour; // Reference to the tour vector
    vector<int> pos;   // pos[node_id] -> index in tour, or -1 if not in tour
    vector<bool> in_tour; // in_tour[node_id] -> true if in tour

    LSHelpers(int n, int k, vector<int>& t) : N(n), K(k), tour(t), pos(n, -1), in_tour(n, false) {
        build();
    }
    
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

// Delta: Inter-route calculation (swap node u and v (not in tour))
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

// Delta: Intra-route (Node exchange) [Swap node u and v]---

static long long delta_intra_exchange_nodes(int i, int j, const LSHelpers& h, const Instance& I) {
    const auto& D = I.D;
    int K = h.K;
    if (i == j) return 0;
    if (j < i) swap(i, j); // ensure i < j by swapping

    int u = h.tour[i];
    int v = h.tour[j];

    int prev_i = h.tour[(i - 1 + K) % K]; // node before u
    int next_i = h.tour[(i + 1) % K]; // node after u
    int prev_j = h.tour[(j - 1 + K) % K]; // node before v
    int next_j = h.tour[(j + 1) % K]; // node after v

    long long delta = 0;

    if (j == i + 1) { // Adjacent nodes

        // Original: (prev_i) -> u -> v -> (next_j)
        // New:      (prev_i) -> v -> u -> (next_j)
        delta = (long long)D[prev_i][v] + D[v][u] + D[u][next_j]
              - D[prev_i][u] - D[u][v] - D[v][next_j];
    } else if (i == 0 && j == K - 1) { // Adjacent at beginning/end of the tour

        // Original: (prev_i=v) -> u -> (next_i) ... (prev_j) -> v -> (next_j=u)
        // We are swapping u (at 0) and v (at K-1)
        // prev_j is node at K-2, next_i is node at 1
        // Original edges: (prev_j) -> v, v -> u, u -> (next_i)
        // New edges:      (prev_j) -> u, u -> v, v -> (next_i)

        delta = (long long)D[prev_j][u] + D[u][v] + D[v][next_i]
              - D[prev_j][v] - D[v][u] - D[u][next_i];
    }
    // Non-adjacent nodes
    else { 
        // Original: (prev_i) -> u -> (next_i) ... (prev_j) -> v -> (next_j)
        // New:      (prev_i) -> v -> (next_i) ... (prev_j) -> u -> (next_j)
        delta = (long long)D[prev_i][v] + D[v][next_i] + D[prev_j][u] + D[u][next_j]
              - D[prev_i][u] - D[u][next_i] - D[prev_j][v] - D[v][next_j];
    }
    return delta;
}

// Delta: Intra-route (Edge exchange) [break and reconnect edges [i,i+1], [j,j+1]]
static long long delta_intra_exchange_edges_2opt(int i, int j, const LSHelpers& h, const Instance& I) {
    const auto& D = I.D;
    int K = h.K;
    if (j < i) swap(i, j); // ensure i < j by swapping

    int u = h.tour[i]; // node at position i
    int v = h.tour[(i + 1) % K]; // node after u
    int x = h.tour[j]; // node at position j
    int y = h.tour[(j + 1) % K]; // node after x

    // No need to swap adjacent edges
    if (v == x || u == y) return 0;

    // Original: (u) -> (v) ... (x) -> (y)
    // New:      (u) -> (x) ... (v) -> (y)

    long long delta = (long long)D[u][x] + D[v][y] - D[u][v] - D[x][y]; // Edge changes only same as before
    return delta;
}

// Applications of moves
static void apply_inter_exchange(LSHelpers& h, int u, int v) {
    int i = h.pos[u];
    
    h.tour[i] = v; // Put v in u's spot
    
    h.pos[u] = -1;
    h.in_tour[u] = false;  //Delete u from tour
    h.pos[v] = i;
    h.in_tour[v] = true;   //Add v to tour
}

static void apply_intra_exchange_nodes(LSHelpers& h, int i, int j) {
    int u = h.tour[i];
    int v = h.tour[j];

    swap(h.tour[i], h.tour[j]);  // swap of nodes
    
    h.pos[u] = j;
    h.pos[v] = i;
}

static void apply_intra_exchange_edges_2opt(LSHelpers& h, int i, int j) {
    int K = h.K;
    if (j < i) swap(i, j);

    int start = (i + 1) % K;
    int end = j;

    // Reversing the segment
    while (start != end) {
        int u = h.tour[start]; // node at position start
        int v = h.tour[end];

        swap(h.tour[start], h.tour[end]); // swap nodes
        h.pos[u] = end; // update position of node u
        h.pos[v] = start; // update position of node v

        start = (start + 1) % K; // move start forward
        if (start == end) break; // check if they met
        end = (end - 1 + K) % K; // move end backward
    }
}

enum IntraMoveType { NODE_EXCHANGE, EDGE_EXCHANGE_2OPT };

// ---- Steep Local Search ----
// Evaluate all moves. Choose the best one ()
vector<int> local_search_steepest(
    const vector<int>& initial_tour, 
    const Instance& I, 
    IntraMoveType intra_mode, 
    mt19937& rng) 
{
    vector<int> tour = initial_tour;
    LSHelpers h(I.N, I.K, tour);
    long long current_obj = objective(tour, I);

    vector<int> nodes_in_tour = tour;
    vector<int> nodes_not_in_tour;
    for (int i = 0; i < I.N; ++i) {
        if (!h.in_tour[i]) {
            nodes_not_in_tour.push_back(i);
        }
    }

    while (true) {
        long long best_delta = 0;
        int best_move_type = -1; // 0: inter, 1: intra
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
        if (intra_mode == NODE_EXCHANGE) {
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
                for (int j = i + 1; j < I.K; ++j) { // Note: j=i+1 is adjacent, delta fn handles this
                    long long delta = delta_intra_exchange_edges_2opt(i, j, h, I);
                     if (delta < best_delta) {
                        best_delta = delta;
                        best_move_type = 1;
                        best_i = i;
                        best_j = j;
                    }
                }
            }
        }
        
        // 3. Apply best move
        if (best_delta < 0) {
            long long old_obj = objective(tour, I); // Debugging
            if (best_move_type == 0) {
                apply_inter_exchange(h, best_u, best_v);
                // Update lists for next iteration
                for (size_t i = 0; i < nodes_in_tour.size(); ++i) {
                    if (nodes_in_tour[i] == best_u) {
                        nodes_in_tour[i] = best_v;
                        break;
                    }
                }
                for (size_t i = 0; i < nodes_not_in_tour.size(); ++i) {
                    if (nodes_not_in_tour[i] == best_v) {
                        nodes_not_in_tour[i] = best_u;
                        break;
                    }
                }

            } else { // move_type == 1
                if (intra_mode == NODE_EXCHANGE) {
                    apply_intra_exchange_nodes(h, best_i, best_j);
                } else {
                    apply_intra_exchange_edges_2opt(h, best_i, best_j);
                }
                // nodes_in_tour list doesn't change, just their order in h.tour
            }
            current_obj += best_delta;
            long long new_obj = objective(tour, I); // Debug
            if (new_obj != old_obj + best_delta) {
                cerr << "FATAL DELTA MISMATCH (STEEP)" << endl; exit(1);
            }
        } else {
            break; // Local optimum reached
        }
    }
    return tour;
}

// Greedy local search
// : Evaluate moves in random order, accept the first improving
vector<int> local_search_greedy(
    const vector<int>& initial_tour, 
    const Instance& I, 
    IntraMoveType intra_mode, 
    mt19937& rng) 
{
    vector<int> tour = initial_tour;
    LSHelpers h(I.N, I.K, tour);
    long long current_obj = objective(tour, I);
    
    // 0: inter {u, v}, 1: intra-node {i, j}, 2: intra-edge {i, j}
    vector<tuple<int, int, int>> moves;

    while (true) {
        bool found_improving = false;

        // 1. Build list of all possible moves
        moves.clear();
        vector<int> nodes_in_tour = tour;
        vector<int> nodes_not_in_tour;
        for (int i = 0; i < I.N; ++i) {
            if (!h.in_tour[i]) {
                nodes_not_in_tour.push_back(i);
            }
        }

        // Add inter-moves
        for (int u : nodes_in_tour) {
            for (int v : nodes_not_in_tour) {
                moves.emplace_back(0, u, v);
            }
        }
        
        // Add intra-moves
        if (intra_mode == NODE_EXCHANGE) {
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
            if (delta < 0) {
                long long old_obj = objective(tour, I); // Debug
                if (type == 0) {
                    apply_inter_exchange(h, a, b);
                } else if (type == 1) {
                    apply_intra_exchange_nodes(h, a, b);
                } else {
                    apply_intra_exchange_edges_2opt(h, a, b);
                }
                current_obj += delta;
                long long new_obj = objective(tour, I); // Debug
                if (new_obj != old_obj + delta) {
                   cerr << "FATAL DELTA MISMATCH (GREEDY)" << endl; exit(1);
                }
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


// LS with List of Moves (LM)
vector<int> local_search_steepest_with_LM(
    const vector<int>& initial_tour,
    const Instance& I,
    mt19937& rng)
{
    vector<int> tour = initial_tour;
    LSHelpers h(I.N, I.K, tour);

    auto rebuild_pos = [&](){
        // LSHelpers keeps pos/in_tour updated when we apply via helpers,
        // but if we ever modify tour directly, rebuild is here.
        // Not needed now; we apply via helpers only.
    };

    vector<SavedMove> LM;// list of improving moves carried forward
    const auto& D = I.D; // distance matrix
    const auto& C = I.cost; // cost vector

    // Evaluate all possible moves and fill LM with improving ones
    auto evaluate_all_and_fill_LM = [&]() -> pair<bool, SavedMove> {
        LM.clear(); // clear list of moves
        long long best_delta = 0; // best improvement found start with 0
        SavedMove best_mv; bool found=false; // best move found

        // Prepare sets
        vector<int> nodes_in_tour = tour; // nodes currently in the tour
        vector<int> nodes_not_in_tour; // nodes currently not in the tour
        nodes_not_in_tour.reserve(I.N - I.K); // reserve space for nodes not in tour
        for (int x = 0; x < I.N; ++x) if (!h.in_tour[x]) nodes_not_in_tour.push_back(x); // fill not-in-tour list

        // ---- INTER (swap one in-tour u with one v_out) ----
        for (int u : nodes_in_tour){ // for each node u in the tour
            int iu = h.pos[u]; // position of u in the tour
            int pu = h.tour[(iu - 1 + h.K) % h.K]; // predecessor of u in the tour
            int nu = h.tour[(iu + 1) % h.K]; // successor of u in the tour
            for (int v_out : nodes_not_in_tour){ // for each node v_out not in the tour
                //same delta as before
                long long delta = (long long)C[v_out] - C[u] 
                                + (long long)D[pu][v_out] + D[v_out][nu]
                                - (long long)D[pu][u] - D[u][nu];
                if (delta < 0){ // improving move
                    SavedMove mv;
                    mv.type = SavedMove::INTER_SWAP; mv.delta = delta; // set move type and delta
                    mv.rem1 = {pu, u}; mv.rem2 = {u, nu}; // edges to remove
                    mv.u = u; mv.v_out = v_out; // nodes involved
                    LM.push_back(mv); // add move to list
                    if (delta < best_delta){ best_delta = delta; best_mv = mv; found=true; } // track best move
                }
            }
        }

        // ---- INTRA 2-OPT (edges [i,i+1] and [j,j+1]) ----
        for (int i = 0; i < h.K; ++i){
            int u = h.tour[i]; // first node of the first edge
            int v = h.tour[(i+1)%h.K]; // second node of the first edge
            for (int j = i+1; j < h.K; ++j){
                int x = h.tour[j]; // first node of the second edge
                int y = h.tour[(j+1)%h.K]; // second node of the second edge
                if (v == x || u == y) continue; // adjacent --> no change

                long long delta = (long long)D[u][x] + D[v][y] - D[u][v] - D[x][y];


                if (delta < 0){ // improving move
                    // forward orientation
                    SavedMove mv;
                    mv.type = SavedMove::INTRA_2OPT;
                    mv.delta = delta; 
                    mv.rem1 = {u, v}; // edges to remove
                    mv.rem2 = {x, y}; // edges to remove
                    mv.i_idx = i; mv.j_idx = j; // indices of edges
                    LM.push_back(mv); // add move to list
                    if (delta < best_delta){ best_delta = delta; best_mv = mv; found=true; } // track best move

                    // *** also add the inverted-orientation variant ***
                    SavedMove mv_inv = mv;
                    mv_inv.rem1 = {v, u}; // edges to remove (inverted)
                    mv_inv.rem2 = {y, x}; // edges to remove (inverted)
                    // (i_idx/j_idx remain the same indices; we’ll recompute at apply-time)
                    LM.push_back(mv_inv); // add move to list
                }
            }
        }

        // we included moves with directed edges (u->v) and (x->y).
        // Inverted edges (v->u) / (y->x) will be considered next iter by the LM rules.

        // Remove the chosen move from LM now (since we will apply it immediately)
        if (found){ 
            // erase first match (by pointer equality is tricky, compare fields)
            for (size_t k=0; k<LM.size(); ++k){
                if (LM[k].type==best_mv.type && LM[k].delta==best_mv.delta &&
                    LM[k].rem1.a==best_mv.rem1.a && LM[k].rem1.b==best_mv.rem1.b &&
                    LM[k].rem2.a==best_mv.rem2.a && LM[k].rem2.b==best_mv.rem2.b &&
                    LM[k].u==best_mv.u && LM[k].v_out==best_mv.v_out){
                    LM.erase(LM.begin()+k);
                    break;
                }
            }
        }
        return {found, best_mv};
    };

    auto try_LM_once = [&]() -> pair<bool, SavedMove> {
        long long best_delta = 0;
        bool found = false;
        SavedMove chosen;

        vector<SavedMove> keep; keep.reserve(LM.size());
        for (auto &mv : LM) {
            if (mv.type == SavedMove::INTER_SWAP) {
                // u must still be in tour; v_out must be outside
                if (h.pos[mv.u] < 0 || h.in_tour[mv.v_out]) {
                    // invalid now -> drop
                    continue;
                }
                // recompute *current* delta
                long long cur_delta = delta_inter_exchange(mv.u, mv.v_out, h, I);
                if (cur_delta < 0) {
                    // prefer most improving
                    if (!found || cur_delta < best_delta) {
                        found = true; best_delta = cur_delta; chosen = mv;
                        // stamp helpful indices (not strictly needed for inter)
                        chosen.i_idx = -1; chosen.j_idx = -1;
                        chosen.delta = cur_delta; // store current delta
                    }
                    keep.push_back(mv); // keep for potential future reuse
                } else {
                    // no longer improving -> drop
                }
                continue;
            }

            // 2-opt reuse: check edges exist and compute current indices
            bool e1_fwd = edge_exists_fwd(h.tour, h.pos, mv.rem1.a, mv.rem1.b);
            bool e1_rev = edge_exists_rev(h.tour, h.pos, mv.rem1.a, mv.rem1.b);
            bool e2_fwd = edge_exists_fwd(h.tour, h.pos, mv.rem2.a, mv.rem2.b);
            bool e2_rev = edge_exists_rev(h.tour, h.pos, mv.rem2.a, mv.rem2.b);

            // if an edge is missing entirely -> drop
            if ((!e1_fwd && !e1_rev) || (!e2_fwd && !e2_rev)) {
                continue;
            }

            // we only allow same relative direction
            bool same_dir = (e1_fwd && e2_fwd) || (e1_rev && e2_rev);
            if (!same_dir) {
                // keep; might become applicable later
                keep.push_back(mv);
                continue;
            }

            // Recover current (i,j) consistent with direction
            int i = e1_fwd ? h.pos[mv.rem1.a] : h.pos[mv.rem1.b];
            int j = e2_fwd ? h.pos[mv.rem2.a] : h.pos[mv.rem2.b];
            if (i < 0 || j < 0) {
                // should not happen if edges exist, but be defensive
                continue;
            }
            if (j < i) std::swap(i, j);

            // Adjacent edges -> no-op; drop
            int v = h.tour[(i + 1) % h.K], x = h.tour[j];
            if (v == x) {
                continue;
            }

            long long cur_delta = delta_intra_exchange_edges_2opt(i, j, h, I);
            if (cur_delta < 0) {
                if (!found || cur_delta < best_delta) {
                    found = true; best_delta = cur_delta; chosen = mv;
                    chosen.i_idx = i; chosen.j_idx = j;  // stamp fresh indices
                    chosen.delta = cur_delta;            // store current delta
                }
                keep.push_back(mv); // keep for future reuse
            } else {
                // not improving anymore -> drop
            }
        }

        LM.swap(keep);

        if (found) {
            // remove the exact chosen instance from LM
            for (size_t k = 0; k < LM.size(); ++k) {
                if (LM[k].type == chosen.type &&
                    LM[k].u == chosen.u && LM[k].v_out == chosen.v_out &&
                    LM[k].rem1.a == chosen.rem1.a && LM[k].rem1.b == chosen.rem1.b &&
                    LM[k].rem2.a == chosen.rem2.a && LM[k].rem2.b == chosen.rem2.b) {
                    LM.erase(LM.begin() + k);
                    break;
                }
            }
            return {true, chosen};
        }
        return {false, {}};
    };


    auto apply_saved_move = [&](const SavedMove& mv){
        if (mv.type == SavedMove::INTER_SWAP){
            apply_inter_exchange(h, mv.u, mv.v_out);
            return;
        }
        // 2-opt: we now *always* have i_idx/j_idx set by either fresh eval or try_LM_once
        if (mv.i_idx >= 0 && mv.j_idx >= 0) {
            int i = mv.i_idx, j = mv.j_idx;
            if (j < i) swap(i, j);
            apply_intra_exchange_edges_2opt(h, i, j);
            return;
        }
        // Fallback should never trigger now, but keep it harmless
        int i = idx_of(h.pos, mv.rem1.a);
        int j = idx_of(h.pos, mv.rem2.a);
        if (i >= 0 && j >= 0) {
            if (j < i) swap(i, j);
            apply_intra_exchange_edges_2opt(h, i, j);
        }
    };



    // === main loop ===
    while (true){
        // 1) First, try to reuse LM
        auto [okLM, mvLM] = try_LM_once();
        if (okLM){
            long long before = objective(tour, I);
            apply_saved_move(mvLM);
            long long after = objective(tour, I);
            // Defensive: the saved delta must still be ≤ actual
            // (can be less improving due to inverted edges equivalence)
            // We only assert improvement.
            if (!(after < before)){
                cerr << "LM move did not improve as expected.\n";
                // allow continue to avoid aborting experiments
            }
            continue; // then loop again (LM may have more)
        }

        // 2) No applicable LM move: evaluate neighborhood and refresh LM
        auto [found, best_now] = evaluate_all_and_fill_LM();
        if (!found) break; // local optimum

        // Recompute actual delta on the current tour & indices (they were stamped)
        long long before = objective(tour, I);
        long long actual_delta = (best_now.type == SavedMove::INTER_SWAP)
            ? delta_inter_exchange(best_now.u, best_now.v_out, h, I)
            : delta_intra_exchange_edges_2opt(best_now.i_idx, best_now.j_idx, h, I);

        // Apply only if still improving
        if (actual_delta < 0) {
            apply_saved_move(best_now);
            long long after = objective(tour, I);
            // Strict check against the recomputed delta
            if (after != before + actual_delta) {
                // Don’t abort; just warn in debug builds
                cerr << "Delta mismatch in fresh eval (applied improving move anyway)." << endl;
            }
        } else {
            // The “best” saved move is no longer improving; drop it and continue the loop
            // (LM already holds other candidates; we’ll rebuild if needed)
            continue;
        }


        // loop; LM holds the rest of improving moves for future reuse
    }
    return tour;
}




// One run of basic local search from a random solution 
// Used inside MSLS and ILS
vector<int> basic_LS_for_meta(const Instance& I, mt19937& rng, double& elapsed_ms) {
    Stopwatch sw;
    auto start_tour = random_solution(I, rng, -1);           // random start
    auto improved   = local_search_steepest_with_LM(start_tour, I, rng);
    elapsed_ms = sw.elapsed_ms();
    return improved;
}

// only used in ils -> basically we run ls from a given starting solution
vector<int> LS_from_start(const Instance& I,
                          const vector<int>& start,
                          mt19937& rng,
                          double& elapsed_ms) {
    Stopwatch sw;
    auto improved = local_search_steepest_with_LM(start, I, rng);
    elapsed_ms = sw.elapsed_ms();
    return improved;
}


// SVG output plots with the way the tour looks like
static string esc(const string& s){ return s; }

// Our RUNNER :))))))))
struct MethodResult {
    string method; // just name of the method (random, nn_end etc)
    long long best_obj = LLONG_MAX; // best obj seen by the method - at the start it is always infintity (it is also stated in each of them but 
    // here it is just for clarity - also it can be updated, lets say that we categorically do not want any node with delta > 200 at start (here
    // delta is the obj value of the candidate node))
    vector<int> best_tour; //just vector of the node ids that gave us the best objective
    vector<long long> all_objs; // for min/max/avg
    vector<double> all_times_ms; // for timing
};


// --- Multiple Start Local Search (MSLS) ---
double MSLS(const Instance& I,
                mt19937& rng,
                MethodResult& MR,
                int outer_runs = 20,
                int starts_per_run = 200)
{
    MR.all_objs.clear();
    MR.all_times_ms.clear();
    MR.best_obj = LLONG_MAX;
    MR.best_tour.clear();

    Stopwatch run_sw;

    for (int r = 0; r < outer_runs; ++r) {
        run_sw.reset();
        long long best_obj_run = LLONG_MAX;
        vector<int> best_tour_run;

        for (int k = 0; k < starts_per_run; ++k) {
            double ls_time_ms = 0.0;
            auto tour = basic_LS_for_meta(I, rng, ls_time_ms);
            long long obj = objective(tour, I);
            if (obj < best_obj_run) {
                best_obj_run = obj;
                best_tour_run = tour;
            }
        }

        double run_time = run_sw.elapsed_ms();
        MR.all_objs.push_back(best_obj_run);
        MR.all_times_ms.push_back(run_time);

        if (best_obj_run < MR.best_obj) {
            MR.best_obj = best_obj_run;
            MR.best_tour = best_tour_run;
        }
    }

    double sum_time = 0.0;
    for (double t : MR.all_times_ms) sum_time += t;
    return sum_time / MR.all_times_ms.size();    // average MSLS running time
}


// --- Iterated Local Search (ILS) ---
// time_budget_ms: average MSLS runtime (per instance)
// ils_runs: 20 (as in assignment)
// ls_runs_per_run: number of basic LS calls in each ILS run (for the report)
void ILS(const Instance& I,
             mt19937& rng,
             MethodResult& MR,
             double time_budget_ms,
             vector<int>& ls_runs_per_run,
             int ils_runs = 20,
             int perturb_moves = 3)
{
    MR.all_objs.clear();
    MR.all_times_ms.clear();
    MR.best_obj = LLONG_MAX;
    MR.best_tour.clear();
    ls_runs_per_run.clear();
    ls_runs_per_run.reserve(ils_runs);

    for (int r = 0; r < ils_runs; ++r) {
        Stopwatch timer;
        int ls_calls = 0;

        // 1) random start + first LS
        auto current = random_solution(I, rng, -1);
        {
            double dummy = 0.0;
            current = LS_from_start(I, current, rng, dummy);
            ++ls_calls;
        }
        long long best_obj_run = objective(current, I);
        vector<int> best_tour_run = current;

        // 2) ILS main loop within time budget
        while (timer.elapsed_ms() < time_budget_ms) {
            auto candidate = best_tour_run;               // perturb best-so-far
            perturb_tour_2opt(candidate, rng, perturb_moves);

            double dummy = 0.0;
            candidate = LS_from_start(I, candidate, rng, dummy);
            ++ls_calls;

            long long cand_obj = objective(candidate, I);
            if (cand_obj < best_obj_run) {
                best_obj_run = cand_obj;
                best_tour_run = candidate;
            }
        }

        double run_time = timer.elapsed_ms();
        MR.all_objs.push_back(best_obj_run);
        MR.all_times_ms.push_back(run_time);
        ls_runs_per_run.push_back(ls_calls);

        if (best_obj_run < MR.best_obj) {
            MR.best_obj = best_obj_run;
            MR.best_tour = best_tour_run;
        }
    }
}


int main(int argc, char** argv) {
    ios::sync_with_stdio(false); 
    cin.tie(nullptr); 

    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <instance1.txt> [instance2.txt ...] "
             << "[--runs-per-start 1] [--random-runs 200] [--seed 106] [--out outdir]\n";
        return 1;
    }

    int runs_per_start = 1; 
    int runs_for_random = 200; 
    unsigned seed = 106; 
    string outdir = "out";
    vector<string> files;

for (int i=1; i<argc; ++i) {
    string a = argv[i];
    if (a == "--runs-per-start" && i+1 < argc) { runs_per_start = stoi(argv[++i]); }
    else if (a == "--random-runs" && i+1 < argc) { runs_for_random = stoi(argv[++i]); }
    else if (a == "--seed" && i+1 < argc) { seed = (unsigned)stoul(argv[++i]); }
    else if (a == "--out" && i+1 < argc) { outdir = argv[++i]; }
    else if (a.rfind("--",0)==0) { cerr << "Unknown flag: " << a << "\n"; return 1; }
    else files.push_back(a);
}

    filesystem::create_directories(outdir);

    mt19937 rng(seed);

    // Prepare summary CSV files - ensures that we don't overwrite the result files
    string short_summary_obj_csv = (filesystem::path(outdir) / "all_results_summary_objectives.csv").string();
    string short_summary_time_csv = (filesystem::path(outdir) / "all_results_summary_times.csv").string();

    static bool headers_written = false;
    if (!headers_written) {
        ofstream SSO(short_summary_obj_csv);
        SSO << "instance,method,summary\n";
        ofstream SST(short_summary_time_csv);
        SST << "instance,method,summary (ms)\n";
        headers_written = true;
    }


    for (const auto& path : files) {
        Instance I;
        if (!read_instance(path, I)) return 2;
        cout << "Instance: " << I.name << "   N=" << I.N << "  K=" << I.K << "\n";


        vector<MethodResult> methods;
        methods.push_back({"CH_RANDOM", LLONG_MAX, {}, {}, {}}); //0

        // MSLS and ILS
        methods.push_back({"MSLS_LS_Steep_Edge_LM", LLONG_MAX, {}, {}, {}}); // 22
        methods.push_back({"ILS_LS_Steep_Edge_LM",  LLONG_MAX, {}, {}, {}}); // 23



        Stopwatch sw;

        // Save random tour for LS
        vector<vector<int>> random_tours;
        vector<vector<int>> greedy_tours;
        random_tours.reserve(runs_for_random);
        greedy_tours.reserve(I.N);

        // RANDOM: 200 total (not per start)
        {
            auto &MR = methods[0];
            for (int r = 0; r < runs_for_random; ++r) {
                //Progress bar
                if ((r + 1) % 5 == 0 || r == runs_for_random - 1) { // Update every 5 runs
                    double progress = (double)(r + 1) / runs_for_random;
                    print_progress(progress, "Running CH_RANDOM...");
                }
                sw.reset();

                auto tour = random_solution(I, rng, -1);
                long long obj = objective(tour, I);
                MR.all_objs.push_back(obj);
                if (obj < MR.best_obj) { MR.best_obj = obj; MR.best_tour = tour; }

                random_tours.push_back(tour);  // Random tour for LS
            }
        }
        cerr << "\nDone." << endl;

        // NN and greedy: 200 per start node
        cout << "Running " << I.N << " greedy starts (for CH and LS)...\n";
        for (int start = 0; start < I.N; ++start) {
            
            // Show progress based on the outer loop (start node)
            double progress = (double)(start + 1) / I.N;
            print_progress(progress, "Greedy Starts (All Methods) Node " + to_string(start + 1) + "/" + to_string(I.N));

            for (int r = 0; r < runs_per_start; ++r) {

                // GREEDY_CYCLE_REGRET2_WEIGHTED (1,1) (9)
                sw.reset();
                auto tour9 = greed_cycle_regret2_weighted(I, start, rng, 1.0, 1.0);
                long long obj9 = objective(tour9, I);
                methods[9].all_objs.push_back(obj9);
                methods[9].all_times_ms.push_back(sw.elapsed_ms());
                if (obj9 < methods[9].best_obj) { methods[9].best_obj = obj9; methods[9].best_tour = tour9; }
                // Save this as the greedy start for LS
                greedy_tours.push_back(tour9);

            }
        }
        cerr << "\nDone." << endl;

        // LOCAL SEARCH from Random Initialization
        std::cout << endl << "Running local search...\n";

        cout << "Running Local Search from " << runs_for_random << " random starts...\n";
        int total_ls_rand_runs = random_tours.size() * 5; // 4 LS variants
        int current_ls_rand_run = 0;
        
        for (size_t i = 0; i < random_tours.size(); ++i) {
            const auto& start_tour = random_tours[i];




        // =====================================================================
        //          MULTIPLE START LOCAL SEARCH (MSLS) + ITERATED LS (ILS)
        // =====================================================================
        cout << "\nRunning MSLS (20 runs, 200 LS calls each)...\n";
        int idx_MSLS = 22;
        int idx_ILS  = 23;

        // MSLS: 20 runs, 200 basic LS runs per MSLS run
        double avg_msls_time_ms = MSLS(I, rng, methods[idx_MSLS], 20, 200);
        cout << "Average MSLS time for instance " << I.name
             << " = " << avg_msls_time_ms << " ms\n";

        // ILS: 20 runs, each with time budget = avg MSLS time
        cout << "Running ILS (20 runs, time budget = avg MSLS time)...\n";
        vector<int> ils_ls_runs;
        ILS(I, rng, methods[idx_ILS], avg_msls_time_ms, ils_ls_runs, 20, 3);

        // Save table with number of basic LS runs per ILS run
        string ils_ls_csv = (filesystem::path(outdir) / (I.name + "_ILS_LS_runs.csv")).string();
        {
            ofstream F(ils_ls_csv);
            F << "instance,run,ls_calls\n";
            for (size_t r = 0; r < ils_ls_runs.size(); ++r) {
                F << I.name << "," << (r + 1) << "," << ils_ls_runs[r] << "\n";
            }

            if (!ils_ls_runs.empty()) {
                int min_calls = *min_element(ils_ls_runs.begin(), ils_ls_runs.end());
                int max_calls = *max_element(ils_ls_runs.begin(), ils_ls_runs.end());
                double sum_calls = 0.0;
                for (int v : ils_ls_runs) sum_calls += v;
                double avg_calls = sum_calls / ils_ls_runs.size();

                F << "SUMMARY,min,max,avg\n";
                F << "," << min_calls << "," << max_calls << "," << avg_calls << "\n";
            }
        }
        cout << "ILS LS-run counts saved to: " << ils_ls_csv << "\n";

    return 0;
}
