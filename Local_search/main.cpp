#include <bits/stdc++.h>
using namespace std;
#include <iomanip>
#include <chrono>

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


// // ------------------- Delta Calculations --------------------------------------
// // Intra-route two nodes exchange (swap positions of the nodes)
long long delta_nodes_exchange(const vector<int>& tour, int i, int j, 
                               const vector<vector<int>>& D) {
    int m = (int)tour.size();
    if (i == j) return 0;
    if (i > j) swap(i, j);  // ensure i < j for better logic
    
    // Special case: adjacent nodes (less computation)
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
} //LOOKS GOOD

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

// NN: append only at end (path -> cycle at end)
vector<int> nn_end_only(const Instance& I, int start, mt19937& rng) {

    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N; 

    vector<char> used(N, 0); // to mark which nodes are already in the path
    vector<int> path; path.reserve(K); // reserve space for K nodes

    path.push_back(start); used[start] = 1; // start node

    while ((int)path.size() < K) {
        int u = path.back(); // last node in the current path
        long long best = LLONG_MAX; // best score for the next node to add first we set it to basically infinity
        vector<int> cand; // candidates for the next node to add

        for (int j = 0; j < N; ++j) if (!used[j]) { // for each node not yet used we evaluate obj value and in cand we store the candidates
            long long delta = (long long)D[u][j] + C[j];
            if (delta < best) { best = delta; cand.assign(1, j); }
            else if (delta == best) cand.push_back(j);
        }
        // random tie break among candidates - the ones that have same obj
        uniform_int_distribution<int> rd(0, (int)cand.size()-1); // random distribution to select among candidates
        int j = cand[rd(rng)]; // select random candidate
        path.push_back(j); // add best candidate to path
        used[j] = 1; // mark it as used
    }
    return path; // scoring adds closing edge
}

// NN: insert anywhere in a PATH 
vector<int> nn_path_insert_anywhere(const Instance& I, int start, mt19937& rng) {
    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N;

    vector<char> used(N, 0);
    vector<int> path; path.reserve(K);

    path.push_back(start); used[start] = 1;

    while ((int)path.size() < K) {
        long long best_val = LLONG_MAX; //same as before we set best score to infinity if we find a better one then we update it 
        //as well as the best node and position (see below)
        int best_j = -1, best_pos = -1; // we track the bestscores to choose at the end of iteration

        for (int j = 0; j < N; ++j) if (!used[j]) { // for each unused node
            // evaluate all positions on the current PATH
            long long local_best = LLONG_MAX; int local_pos = -1; // best position for this j - to witch node it will be inserted - BUT
            //only if it will give us the best score overall - so we compare it to best_val below
            int m = (int)path.size(); // current path size

            if (m == 1) { // special case: only one node in path
                long long delta = D[j][path[0]]; 
                if (delta < local_best) { local_best = delta; local_pos = 0; } // before 0 == prepend; also append is same here
            } 
            else {
                // prepend -- insert before path[0], so at the start of the path
                {
                    long long delta = D[j][path[0]];
                    if (delta < local_best) { local_best = delta; local_pos = 0; }
                }
                // internal positions between t and t+1 -- so anywhere in the middle of the path
                for (int t = 0; t+1 < m; ++t) {
                    int u = path[t], v = path[t+1];
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v]; // cost change if we insert j between u and v since now we remove u->v and add u->j + j->v
                    if (delta < local_best) { local_best = delta; local_pos = t+1; }
                }
                // append -- insert after path[m-1] -- so at the end of the path
                {
                    long long delta = D[path[m-1]][j];
                    if (delta < local_best) { local_best = delta; local_pos = m; }
                }
            }

            long long total = local_best + C[j]; // total cost change if we insert j at best position found for it
            // compare to overall best
            if (total < best_val) { best_val = total; best_j = j; best_pos = local_pos; } // if it is better than overall best we update everything
            else if (total == best_val) {
                // random tie break
                if (uniform_int_distribution<int>(0,1)(rng)) { best_j = j; best_pos = local_pos; }
            }
        }
        insert_at(path, best_pos, best_j); // insert best_j at best_pos in path
        used[best_j] = 1;
    }
    return path; // scoring adds closing edge
}

// --------------------- GREEDY REGRET HEURISTICS - NN_INSERT_ANYWHERE ---------------------

vector<int> nn_path_insert_anywhere_regret2(const Instance& I, int start, mt19937& rng) {
    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N;
    vector<char> used(N, 0);  // to mark used nodes as before
    vector<int> path; path.reserve(K); // reserve space for K elements
    path.push_back(start); used[start] = 1; // marking start node as used 

    while ((int)path.size() < K) {
        int m = (int)path.size();

        long long best_reg = LLONG_MIN;    // we MAXIMIZE regret = best2 - best1 so start with setting is a minimal val
        long long best1_tie = LLONG_MAX; // tie-break: prefer smaller best1
        int pick_j = -1, pick_pos = -1; // node j to insert and position to insert at

        for (int j = 0; j < N; ++j) if (!used[j]) {
            // gather all insertion costs (total = delta + C[j]) and positions
            vector<pair<long long,int>> ins; // (total_cost, position_index)
            ins.reserve(max(2, m+1));

            if (m == 1) {
                // prepend and append are identical when only one node exists
                long long tot = (long long)D[j][path[0]] + C[j];
                ins.push_back({tot, 0});        // before 0
                ins.push_back({tot, 1});        // after 0
            } else {
                // prepend (before index 0)
                ins.push_back({(long long)D[j][path[0]] + C[j], 0});
                // internal between t and t+1  => position (t+1)
                for (int t = 0; t+1 < m; ++t) {
                    int u = path[t], v = path[t+1];
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    ins.push_back({delta + C[j], t+1});
                }
                // append (after last) => position m
                ins.push_back({(long long)D[path[m-1]][j] + C[j], m});
            }

            // best1 and best2
            sort(ins.begin(), ins.end(), [](auto& a, auto& b){ return a.first < b.first; });
            long long best1 = ins[0].first;
            int pos_best1 = ins[0].second;
            long long best2 = (ins.size() >= 2 ? ins[1].first : ins[0].first);
            long long reg = best2 - best1; // getting the regret value - just a diff between 2 best insertion objs

            // choose by max regret, then smaller best1, then random
            bool take = false;
            if (reg > best_reg) take = true;
            else if (reg == best_reg) { // tie-break - if same picking at random
                if (best1 < best1_tie) take = true;
                else if (best1 == best1_tie && uniform_int_distribution<int>(0,1)(rng)) take = true;
            }
            if (take) {
                best_reg = reg;
                best1_tie = best1;
                pick_j = j;
                pick_pos = pos_best1;
            }
        }

        path.insert(path.begin() + pick_pos, pick_j);
        used[pick_j] = 1;
    }
    return path; 
}


// ===== Greedy WEIGHTED (2-regret + best1) on an OPEN PATH =====
vector<int> nn_path_insert_anywhere_regret2_weighted(const Instance& I, int start, mt19937& rng,
                                                  double w_reg = 1.0, double w_cost = 1.0) {
    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N;
    vector<char> used(N, 0);
    vector<int> path; path.reserve(K);
    path.push_back(start); used[start] = 1;

    while ((int)path.size() < K) {
        int m = (int)path.size();

        double best_score = -1e300;
        long long best1_tie = LLONG_MAX;
        int pick_j = -1, pick_pos = -1;

        for (int j = 0; j < N; ++j) if (!used[j]) {
            vector<long long> costs;
            vector<int> pos;
            if (m == 1) {
                long long tot = (long long)D[j][path[0]] + C[j];
                costs.push_back(tot); pos.push_back(0); 
                costs.push_back(tot); pos.push_back(1); 
            } else {
                // prepend
                costs.push_back((long long)D[j][path[0]] + C[j]); pos.push_back(0);
                // internal
                for (int t = 0; t+1 < m; ++t) {
                    int u = path[t], v = path[t+1];
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    costs.push_back(delta + C[j]); pos.push_back(t+1);
                }
                // append
                costs.push_back((long long)D[path[m-1]][j] + C[j]); pos.push_back(m);
            }

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
                best1_tie = best1;
                pick_j = j;
                pick_pos = pos[b1];
            } else if (fabs(score - best_score) < 1e-12) {
                if (best1 < best1_tie ||
                    (best1 == best1_tie && uniform_int_distribution<int>(0,1)(rng))) {
                    best1_tie = best1;
                    pick_j = j;
                    pick_pos = pos[b1];
                }
            }
        }

        path.insert(path.begin() + pick_pos, pick_j);
        used[pick_j] = 1;
    }
    return path; 
}


// Greedy Cycle (Cheapest Insertion) — build a cycle directly by inserting nodes at cheapest position
vector<int> greedy_cycle_cheapest_insertion(const Instance& I, int start, mt19937& rng) {
    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N;

    vector<char> used(N, 0); // to mark used nodes
    vector<int> cyc; cyc.reserve(K); // reserve space for K nodes
    cyc.push_back(start); used[start] = 1; // start node

    while ((int)cyc.size() < K) { // while we need more nodes

        long long best_val = LLONG_MAX; // best score for next insertion starting with infinity as before

        int best_j = -1, best_insert_after = -1; // insert between cyc[i] and cyc[(i+1)%m]

        int m = (int)cyc.size();
        for (int j = 0; j < N; ++j) if (!used[j]) { // for each unused yet node
            if (m == 1) { // special case: only one node in cycle
                int s = cyc[0]; 
                long long delta = 2LL * D[s][j]; // (s->j + j->s) basically 2 node cycle
                long long total = delta + C[j]; // adding cost of node j to the obj
                if (total < best_val) { best_val = total; best_j = j; best_insert_after = 0; } // inserting after position 0 - so right after start
                else if (total == best_val && uniform_int_distribution<int>(0,1)(rng)) { //tie break
                    best_j = j; best_insert_after = 0;
                }
            } else { // general case: m >= 2 nodes in cycle
                // try all m possible insertion positions
                for (int i = 0; i < m; ++i) { 
                    int u = cyc[i], v = cyc[(i+1)%m]; // edge u->v that will be replaced by u->j + j->v
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v]; // cost change if we insert j between u and v
                    long long total = delta + C[j]; // total cost change including cost of node j
                    if (total < best_val) { best_val = total; best_j = j; best_insert_after = i; } // if it is better than overall best we update everything
                    else if (total == best_val && uniform_int_distribution<int>(0,1)(rng)) {
                        best_j = j; best_insert_after = i;
                    }
                }
            }
        }
        // insert after position best_insert_after
        cyc.insert(cyc.begin() + (best_insert_after + 1), best_j); // insert best_j after best_insert_after
        used[best_j] = 1;
    }
    return cyc;
}



// --------------------- GREEDY REGRET HEURISTICS - GREEDY CYCLE ---------------------

// Greedy 2-regret on a cycle (costs include distance delta + C[j])
vector<int> greed_cycle_regret2(const Instance& I, int start, mt19937& rng) {
    const auto& D = I.D; const auto& C = I.cost; int K = I.K, N = I.N;
    vector<char> used(N, 0);
    vector<int> cyc; cyc.reserve(K);
    cyc.push_back(start); used[start] = 1;

    while ((int)cyc.size() < K) {
        int m = (int)cyc.size();

        long long best_score = LLONG_MIN;      // we MAXIMIZE regret
        int best_j = -1, best_edge_i = -1;     // insert between cyc[i] and cyc[(i+1)%m]
        long long best1_for_pick = 0;          // keep best1(j) for chosen j (for tie-break)

        for (int j = 0; j < N; ++j) if (!used[j]) {
            vector<pair<long long,int>> ins; 
            ins.reserve(max(1, m));
            if (m == 1) {
                int s = cyc[0];
                long long total = 2LL * D[s][j] + C[j]; // two edges + node cost
                ins.push_back({total, 0});
            } else {
                for (int i = 0; i < m; ++i) {
                    int u = cyc[i], v = cyc[(i+1)%m];
                    long long delta = (long long)D[u][j] + D[j][v] - D[u][v];
                    long long total = delta + C[j];
                    ins.push_back({total, i});
                }
            }
            // find best1 and best2
            sort(ins.begin(), ins.end(), [](auto& a, auto& b){ return a.first < b.first; });
            long long best1 = ins[0].first;
            int edge_for_best1 = ins[0].second;
            long long best2 = (ins.size() >= 2 ? ins[1].first : ins[0].first);
            long long regret = best2 - best1; 

            // selection: maximize regret; tie-break by smaller best1; then random
            bool take = false;
            if (regret > best_score) take = true;
            else if (regret == best_score) {
                if (best1 < best1_for_pick) take = true;
                else if (best1 == best1_for_pick && uniform_int_distribution<int>(0,1)(rng)) take = true;
            }
            if (take) {
                best_score = regret;
                best_j = j;
                best_edge_i = edge_for_best1;
                best1_for_pick = best1;
            }
        }

        cyc.insert(cyc.begin() + (best_edge_i + 1), best_j);
        used[best_j] = 1;
    }
    return cyc;
}


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

// Delta: Inter-route calculation (swap node u and v (not in tour
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
    } else if (i == 0 && j == K - 1) { // Adjacent at beginning/end of the tour
        // Original: (prev_i=v) -> u -> (next_i) ... (prev_j) -> v -> (next_j=u)
        // We are swapping u (at 0) and v (at K-1)
        // prev_j is node at K-2, next_i is node at 1
        // Original edges: (prev_j) -> v, v -> u, u -> (next_i)
        // New edges:      (prev_j) -> u, u -> v, v -> (next_i)
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

// Delta: Intra-route (Edge exchange) [break and reconnect edges [i,i+1], [j,j+1]]
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

    swap(h.tour[i], h.tour[j]);  //Perform simple swap of nodes
    
    h.pos[u] = j;
    h.pos[v] = i;
}

static void apply_intra_exchange_edges_2opt(LSHelpers& h, int i, int j) {
    int K = h.K;
    if (j < i) swap(i, j);

    int start = (i + 1) % K;
    int end = j;

    // Reverse the segment
    while (start != end) {
        int u = h.tour[start];
        int v = h.tour[end];

        swap(h.tour[start], h.tour[end]);
        h.pos[u] = end;
        h.pos[v] = start;

        start = (start + 1) % K;
        if (start == end) break;
        end = (end - 1 + K) % K;
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


// SVG output plots with the way the tour looks like
static string esc(const string& s){ return s; }

void save_svg(const Instance& I, const vector<int>& tour, const string& out_path) {
    if (tour.empty()) return;
    int W = 1000, H = 1000, margin = 40;

    int minx = INT_MAX, maxx = INT_MIN, miny = INT_MAX, maxy = INT_MIN; // bounding box of points
    for (auto&p: I.pts) { minx=min(minx,p.x); maxx=max(maxx,p.x); miny=min(miny,p.y); maxy=max(maxy,p.y); } 
    double sx = (maxx==minx) ? 1.0 : (double)(W-2*margin)/(maxx-minx); //scale
    double sy = (maxy==miny) ? 1.0 : (double)(H-2*margin)/(maxy-miny);

    auto X = [&](int x){ return margin + (x - minx) * sx; }; // scale x
    auto Y = [&](int y){ return margin + (y - miny) * sy; }; // scale y

    int minC = *min_element(I.cost.begin(), I.cost.end()); // min and max cost for scaling
    int maxC = *max_element(I.cost.begin(), I.cost.end()); // max cost for scaling
    auto rad = [&](int c){ // just for the node size
        if (maxC==minC) return 6.0;
        double t = (double)(c - minC) / (maxC - minC);
        return 4.0 + 10.0 * t; // 4..14 px
    };
    // auto grey = [&](int c){ // grey scale for node color
    //     if (maxC==minC) return 30;
    //     double t = (double)(c - minC) / (maxC - minC);
    //     return (int)(50 + 180*t); // 50..230
    // };

    auto green = [&](int c){ //cahnged since i am quite blind and i want to see the colors better
        if (maxC == minC) return make_tuple(0, 200, 0);
        double t = (double)(c - minC) / (maxC - minC);
        int r = (int)(255 * t);          // red increases with cost
        int g = (int)(255 * (1 - t));    // green decreases with cost
        int b = 50;
        return make_tuple(r, g, b);
    };



    ofstream out(out_path);
    out << "<svg xmlns='http://www.w3.org/2000/svg' width='"<<W<<"' height='"<<H<<"'>\n"; 
    out << "<rect x='0' y='0' width='100%' height='100%' fill='white'/>\n";

    // drawing tour edges
    out << "<g stroke='black' stroke-width='1' fill='none'>\n";
    for (int i = 0; i < (int)tour.size(); ++i) {
        int u = tour[i], v = tour[(i+1)%tour.size()];
        out << "<line x1='"<<X(I.pts[u].x)<<"' y1='"<<Y(I.pts[u].y)
            <<"' x2='"<<X(I.pts[v].x)<<"' y2='"<<Y(I.pts[v].y)<<"'/>\n";
    }
    out << "</g>\n";

    // drawing nodes (all nodes; highlight selected)
    out << "<g>\n";
    unordered_set<int> sel(tour.begin(), tour.end());
    for (int i = 0; i < I.N; ++i) {
        double cx = X(I.pts[i].x), cy = Y(I.pts[i].y);
        // int g = grey(I.cost[i]);
        // if (sel.count(i)) {
        //     out << "<circle cx='"<<cx<<"' cy='"<<cy<<"' r='"<<rad(I.cost[i])
        //         <<"' fill='rgb("<<g<<","<<g<<","<<g<<")' stroke='black' stroke-width='1'/>\n";
        // } 
        auto [r, g, b] = green(I.cost[i]);
        if (sel.count(i)) {
            out << "<circle cx='"<<cx<<"' cy='"<<cy<<"' r='"<<rad(I.cost[i])
                <<"' fill='rgb("<<r<<","<<g<<","<<b<<")' stroke='black' stroke-width='1'/>\n";
        } else {
            out << "<circle cx='"<<cx<<"' cy='"<<cy<<"' r='3' fill='none' stroke='gray' />\n";  
        }
        // index label for selected
        if (sel.count(i)) {
            out << "<text x='"<<cx+6<<"' y='"<<cy-6<<"' font-size='10' fill='black'>"<<i<<"</text>\n";
        }
    }
    out << "</g>\n";
    out << "</svg>\n";
}

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
        methods.push_back({"CH_NN_END_ONLY", LLONG_MAX, {}, {}, {}}); //1
        methods.push_back({"CH_NN_INSERT_ANYWHERE_PATH", LLONG_MAX, {}, {}, {}}); //2
        methods.push_back({"CH_GREEDY_CYCLE_CHEAPEST_INSERTION", LLONG_MAX, {}, {}, {}}); //3
        methods.push_back({"CH_NN_PATH_INSERT_ANYWHERE_REGRET2", LLONG_MAX, {}, {}, {}});  //4
        methods.push_back({"CH_NN_PATH_INSERT_ANYWHERE_REGRET2_WEIGHTED_reg1_objchange_1", LLONG_MAX, {}, {}, {}}); //5
        methods.push_back({"CH_NN_PATH_INSERT_ANYWHERE_REGRET2_WEIGHTED_reg2_objchange_1", LLONG_MAX, {}, {}, {}}); //6
        methods.push_back({"CH_NN_PATH_INSERT_ANYWHERE_REGRET2_WEIGHTED_reg1_objchange_2", LLONG_MAX, {}, {}, {}}); //7
        methods.push_back({"CH_GREEDY_CYCLE_REGRET2", LLONG_MAX, {}, {}, {}}); //8
        methods.push_back({"CH_GREEDY_CYCLE_REGRET2_WEIGHTED_reg1_objchange_1", LLONG_MAX, {}, {}, {}}); //9
        methods.push_back({"CH_GREEDY_CYCLE_REGRET2_WEIGHTED_reg2_objchange_1", LLONG_MAX, {}, {}, {}}); //10
        methods.push_back({"CH_GREEDY_CYCLE_REGRET2_WEIGHTED_reg1_objchange_2", LLONG_MAX, {}, {}, {}}); //11

        // Local Search Methods
        // Start: Random, Intra: Node Ex
        methods.push_back({"LS_Rand_Steep_Node", LLONG_MAX, {}, {}, {}}); // 12
        methods.push_back({"LS_Rand_Greedy_Node", LLONG_MAX, {}, {}, {}}); // 13
        // Start: Random, Intra: Edge Ex
        methods.push_back({"LS_Rand_Steep_Edge", LLONG_MAX, {}, {}, {}}); // 14
        methods.push_back({"LS_Rand_Greedy_Edge", LLONG_MAX, {}, {}, {}}); // 15
        // Start: Greedy, Intra: Node Ex
        methods.push_back({"LS_Greedy_Steep_Node", LLONG_MAX, {}, {}, {}}); // 16
        methods.push_back({"LS_Greedy_Greedy_Node", LLONG_MAX, {}, {}, {}}); // 17
        // Start: Greedy, Intra: Edge Ex
        methods.push_back({"LS_Greedy_Steep_Edge", LLONG_MAX, {}, {}, {}}); // 18
        methods.push_back({"LS_Greedy_Greedy_Edge", LLONG_MAX, {}, {}, {}}); // 19

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
                // NN_END_ONLY (1)
                sw.reset();
                auto tour1 = nn_end_only(I, start, rng);
                long long obj1 = objective(tour1, I);
                methods[1].all_objs.push_back(obj1);
                methods[1].all_times_ms.push_back(sw.elapsed_ms());
                if (obj1 < methods[1].best_obj) { methods[1].best_obj = obj1; methods[1].best_tour = tour1; }

                // NN_INSERT_ANYWHERE_PATH (2)
                sw.reset();
                auto tour2 = nn_path_insert_anywhere(I, start, rng);
                long long obj2 = objective(tour2, I);
                methods[2].all_objs.push_back(obj2);
                methods[2].all_times_ms.push_back(sw.elapsed_ms());
                if (obj2 < methods[2].best_obj) { methods[2].best_obj = obj2; methods[2].best_tour = tour2; }

                // GREEDY_CYCLE_CHEAPEST_INSERTION (3)
                sw.reset();
                auto tour3 = greedy_cycle_cheapest_insertion(I, start, rng);
                long long obj3 = objective(tour3, I);
                methods[3].all_objs.push_back(obj3);
                methods[3].all_times_ms.push_back(sw.elapsed_ms());
                if (obj3 < methods[3].best_obj) { methods[3].best_obj = obj3; methods[3].best_tour = tour3; }

                // PATH_REGRET2 (4)
                sw.reset();
                auto tour4 = nn_path_insert_anywhere_regret2(I, start, rng);
                long long obj4 = objective(tour4, I);
                methods[4].all_objs.push_back(obj4);
                methods[4].all_times_ms.push_back(sw.elapsed_ms());
                if (obj4 < methods[4].best_obj) { methods[4].best_obj = obj4; methods[4].best_tour = tour4; }
                
                // PATH_REGRET2_WEIGHTED (1,1) (5)
                sw.reset();
                auto tour5 = nn_path_insert_anywhere_regret2_weighted(I, start, rng, 1.0, 1.0);
                long long obj5 = objective(tour5, I);
                methods[5].all_objs.push_back(obj5);
                methods[5].all_times_ms.push_back(sw.elapsed_ms());
                if (obj5 < methods[5].best_obj) { methods[5].best_obj = obj5; methods[5].best_tour = tour5; }
                
                // PATH_REGRET2_WEIGHTED (2,1) (6)
                sw.reset();
                auto tour6 = nn_path_insert_anywhere_regret2_weighted(I, start, rng, 2.0, 1.0);
                long long obj6 = objective(tour6, I);
                methods[6].all_objs.push_back(obj6);
                methods[6].all_times_ms.push_back(sw.elapsed_ms());
                if (obj6 < methods[6].best_obj) { methods[6].best_obj = obj6; methods[6].best_tour = tour6; }
                
                // PATH_REGRET2_WEIGHTED (1,2) (7)
                sw.reset();
                auto tour7 = nn_path_insert_anywhere_regret2_weighted(I, start, rng, 1.0, 2.0);
                long long obj7 = objective(tour7, I);
                methods[7].all_objs.push_back(obj7);
                methods[7].all_times_ms.push_back(sw.elapsed_ms());
                if (obj7 < methods[7].best_obj) { methods[7].best_obj = obj7; methods[7].best_tour = tour7; }

                // GREEDY_CYCLE_REGRET2 (8)
                sw.reset();
                auto tour8 = greed_cycle_regret2(I, start, rng);
                long long obj8 = objective(tour8, I);
                methods[8].all_objs.push_back(obj8);
                methods[8].all_times_ms.push_back(sw.elapsed_ms());
                if (obj8 < methods[8].best_obj) { methods[8].best_obj = obj8; methods[8].best_tour = tour8; }
                
                // GREEDY_CYCLE_REGRET2_WEIGHTED (1,1) (9)
                sw.reset();
                auto tour9 = greed_cycle_regret2_weighted(I, start, rng, 1.0, 1.0);
                long long obj9 = objective(tour9, I);
                methods[9].all_objs.push_back(obj9);
                methods[9].all_times_ms.push_back(sw.elapsed_ms());
                if (obj9 < methods[9].best_obj) { methods[9].best_obj = obj9; methods[9].best_tour = tour9; }
                // Save this as the greedy start for LS
                greedy_tours.push_back(tour9);

                // GREEDY_CYCLE_REGRET2_WEIGHTED (2,1) (10)
                sw.reset();
                auto tour10 = greed_cycle_regret2_weighted(I, start, rng, 2.0, 1.0);
                long long obj10 = objective(tour10, I);
                methods[10].all_objs.push_back(obj10);
                methods[10].all_times_ms.push_back(sw.elapsed_ms());
                if (obj10 < methods[10].best_obj) { methods[10].best_obj = obj10; methods[10].best_tour = tour10; }
                
                // GREEDY_CYCLE_REGRET2_WEIGHTED (1,2) (11)
                sw.reset();
                auto tour11 = greed_cycle_regret2_weighted(I, start, rng, 1.0, 2.0);
                long long obj11 = objective(tour11, I);
                methods[11].all_objs.push_back(obj11);
                methods[11].all_times_ms.push_back(sw.elapsed_ms());
                if (obj11 < methods[11].best_obj) { methods[11].best_obj = obj11; methods[11].best_tour = tour11; }
            }
        }
        cerr << "\nDone." << endl;

        // LOCAL SEARCH from Random Initialization
        std::cout << endl << "Running local search...\n";

        cout << "Running Local Search from " << runs_for_random << " random starts...\n";
        int total_ls_rand_runs = random_tours.size() * 4; // 4 LS variants
        int current_ls_rand_run = 0;
        
        for (size_t i = 0; i < random_tours.size(); ++i) {
            const auto& start_tour = random_tours[i];

            // LS_Rand_Steep_Node (12)
            current_ls_rand_run++;
            print_progress((double)current_ls_rand_run / total_ls_rand_runs, 
                           "LS from Random (Steep, Node) " + to_string(i + 1) + "/" + to_string(random_tours.size()));
            sw.reset();
            auto tour12 = local_search_steepest(start_tour, I, NODE_EXCHANGE, rng);
            long long obj12 = objective(tour12, I);
            methods[12].all_objs.push_back(obj12);
            methods[12].all_times_ms.push_back(sw.elapsed_ms());
            if (obj12 < methods[12].best_obj) { methods[12].best_obj = obj12; methods[12].best_tour = tour12; }

            // LS_Rand_Greedy_Node (13)
            current_ls_rand_run++;
            print_progress((double)current_ls_rand_run / total_ls_rand_runs,
                           "LS from Random (Greedy, Node) " + to_string(i + 1) + "/" + to_string(random_tours.size()));
            sw.reset();
            auto tour13 = local_search_greedy(start_tour, I, NODE_EXCHANGE, rng);
            long long obj13 = objective(tour13, I);
            methods[13].all_objs.push_back(obj13);
            methods[13].all_times_ms.push_back(sw.elapsed_ms());
            if (obj13 < methods[13].best_obj) { methods[13].best_obj = obj13; methods[13].best_tour = tour13; }
            
            // LS_Rand_Steep_Edge (14)
            current_ls_rand_run++;
            print_progress((double)current_ls_rand_run / total_ls_rand_runs,
                           "LS from Random (Steep, Edge) " + to_string(i + 1) + "/" + to_string(random_tours.size()));
            sw.reset();
            auto tour14 = local_search_steepest(start_tour, I, EDGE_EXCHANGE_2OPT, rng);
            long long obj14 = objective(tour14, I);
            methods[14].all_objs.push_back(obj14);
            methods[14].all_times_ms.push_back(sw.elapsed_ms());
            if (obj14 < methods[14].best_obj) { methods[14].best_obj = obj14; methods[14].best_tour = tour14; }

            // LS_Rand_Greedy_Edge (15)
            current_ls_rand_run++;
            print_progress((double)current_ls_rand_run / total_ls_rand_runs,
                           "LS from Random (Greedy, Edge) " + to_string(i + 1) + "/" + to_string(random_tours.size()));
            sw.reset();
            auto tour15 = local_search_greedy(start_tour, I, EDGE_EXCHANGE_2OPT, rng);
            long long obj15 = objective(tour15, I);
            methods[15].all_objs.push_back(obj15);
            methods[15].all_times_ms.push_back(sw.elapsed_ms());
            if (obj15 < methods[15].best_obj) { methods[15].best_obj = obj15; methods[15].best_tour = tour15; }
        }
        cerr << "\nDone." << endl;


        // LOCAL SEARCH from Greedy Initialization
        cout << "Running Local Search from " << greedy_tours.size() << " greedy starts...\n";
        int total_ls_greedy_runs = greedy_tours.size() * 4; // 4 LS variants
        int current_ls_greedy_run = 0;
        
        for(size_t i = 0; i < greedy_tours.size(); ++i) {
            const auto& start_tour = greedy_tours[i];
            
            // LS_Greedy_Steep_Node (16)
            current_ls_greedy_run++;
            print_progress((double)current_ls_greedy_run / total_ls_greedy_runs,
                           "LS from Greedy (Steep, Node) " + to_string(i + 1) + "/" + to_string(greedy_tours.size()));
            sw.reset();
            auto tour16 = local_search_steepest(start_tour, I, NODE_EXCHANGE, rng);
            long long obj16 = objective(tour16, I);
            methods[16].all_objs.push_back(obj16);
            methods[16].all_times_ms.push_back(sw.elapsed_ms());
            if (obj16 < methods[16].best_obj) { methods[16].best_obj = obj16; methods[16].best_tour = tour16; }

            // LS_Greedy_Greedy_Node (17)
            current_ls_greedy_run++;
            print_progress((double)current_ls_greedy_run / total_ls_greedy_runs,
                           "LS from Greedy (Greedy, Node) " + to_string(i + 1) + "/" + to_string(greedy_tours.size()));
            sw.reset();
            auto tour17 = local_search_greedy(start_tour, I, NODE_EXCHANGE, rng);
            long long obj17 = objective(tour17, I);
            methods[17].all_objs.push_back(obj17);
            methods[17].all_times_ms.push_back(sw.elapsed_ms());
            if (obj17 < methods[17].best_obj) { methods[17].best_obj = obj17; methods[17].best_tour = tour17; }
            
            // LS_Greedy_Steep_Edge (18)
            current_ls_greedy_run++;
            print_progress((double)current_ls_greedy_run / total_ls_greedy_runs,
                           "LS from Greedy (Steep, Edge) " + to_string(i + 1) + "/" + to_string(greedy_tours.size()));
            sw.reset();
            auto tour18 = local_search_steepest(start_tour, I, EDGE_EXCHANGE_2OPT, rng);
            long long obj18 = objective(tour18, I);
            methods[18].all_objs.push_back(obj18);
            methods[18].all_times_ms.push_back(sw.elapsed_ms());
            if (obj18 < methods[18].best_obj) { methods[18].best_obj = obj18; methods[18].best_tour = tour18; }

            // LS_Greedy_Greedy_Edge (19)
            current_ls_greedy_run++;
            print_progress((double)current_ls_greedy_run / total_ls_greedy_runs,
                           "LS from Greedy (Greedy, Edge) " + to_string(i + 1) + "/" + to_string(greedy_tours.size()));
            sw.reset();
            auto tour19 = local_search_greedy(start_tour, I, EDGE_EXCHANGE_2OPT, rng);
            long long obj19 = objective(tour19, I);
            methods[19].all_objs.push_back(obj19);
            methods[19].all_times_ms.push_back(sw.elapsed_ms());
            if (obj19 < methods[19].best_obj) { methods[19].best_obj = obj19; methods[19].best_tour = tour19; }
        }
        cerr << "\nDone." << endl;

/////
        // --- Summaries + Best Tours + SVGs ---
        string summary_csv = (filesystem::path(outdir) / (I.name + "_results_summary.csv")).string();
        ofstream S(summary_csv);
        S << "instance,method,runs,min_obj,max_obj,avg_obj,best_obj,min_time_ms,max_time_ms,avg_time_ms\n";
        
        // Open summary files in append mode
        ofstream SSO(short_summary_obj_csv, ios::app);
        ofstream SST(short_summary_time_csv, ios::app);


        for (auto &MR : methods) {
            if (MR.all_objs.empty()) continue; // Skip methods that didn't run

            if (!MR.best_tour.empty()) {
                string why;
                bool ok = check_solution(MR.best_tour, I, &why);
                if (!ok) cerr << "WARNING: best tour for " << MR.method << " failed check: " << why << "\n";
            }
            
            // Objective stats
            long long mn_obj = LLONG_MAX, mx_obj = LLONG_MIN; 
            long double sum_obj = 0;
            for (auto v : MR.all_objs) { mn_obj = min(mn_obj, v); mx_obj = max(mx_obj, v); sum_obj += v; }
            long double avg_obj = sum_obj / (long double)MR.all_objs.size();

            // Time stats
            double mn_time = 1e300, mx_time = -1e300, sum_time = 0;
            for (auto t : MR.all_times_ms) { mn_time = min(mn_time, t); mx_time = max(mx_time, t); sum_time += t; }
            double avg_time = sum_time / (double)MR.all_times_ms.size();


            // Instance-specific summary
            S << I.name << "," << MR.method << "," << MR.all_objs.size() << ","
              << mn_obj << "," << mx_obj << "," << (long long)(avg_obj + 0.5) << "," << MR.best_obj << ","
              << mn_time << "," << mx_time << "," << avg_time << "\n";

            // Global summary (Objectives)
            SSO << I.name << "," << MR.method << ","
               << (long long)(avg_obj + 0.5)
               << " (" << mn_obj << " ; " << mx_obj << ")"
               << "\n";

            // Global summary (Times)
            SST << I.name << "," << MR.method << ","
               << fixed << setprecision(3) << avg_time
               << " (" << mn_time << " ; " << mx_time << ")"
               << "\n";


            // best tour list (with 0-based indices)
            if (!MR.best_tour.empty()) {
                string best_txt = (filesystem::path(outdir) / (I.name + "_best_" + MR.method + ".txt")).string();
                ofstream B(best_txt);
                for (int i = 0; i < (int)MR.best_tour.size(); ++i) {
                    if (i) B << " ";
                    B << MR.best_tour[i];
                }
                B << "\n";

                
                string svg = (filesystem::path(outdir) / (I.name + "_best_" + MR.method + ".svg")).string();
                save_svg(I, MR.best_tour, svg);
            }
        }
        cout << "Wrote: " << summary_csv << " and best tours/SVGs in " << outdir << "\n";
        cout << "Appended to: " << short_summary_obj_csv << " and " << short_summary_time_csv << "\n";
    }
    return 0;
}
