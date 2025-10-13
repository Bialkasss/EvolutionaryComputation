#include <bits/stdc++.h>
using namespace std;

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
            out << "<circle cx='"<<cx<<"' cy='"<<cy<<"' r='3' fill='none' stroke='lightgray' />\n";
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
};

int main(int argc, char** argv) {
    ios::sync_with_stdio(false); 
    cin.tie(nullptr); 

    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <instance1.txt> [instance2.txt ...] "
             << "[--runs 200] [--seed 106] [--out outdir]\n";
        return 1;
    }

    int runs_per_start = 200; // number of runs per start node (for RANDOM it is total, not per start)
    unsigned seed = 106; 
    string outdir = "out";
    vector<string> files;

    for (int i=1; i<argc; ++i) {
        string a = argv[i];
        if (a == "--runs" && i+1 < argc) { runs_per_start = stoi(argv[++i]); }
        else if (a == "--seed" && i+1 < argc) { seed = (unsigned)stoul(argv[++i]); }
        else if (a == "--out" && i+1 < argc) { outdir = argv[++i]; }
        else if (a.rfind("--",0)==0) { cerr << "Unknown flag: " << a << "\n"; return 1; }
        else files.push_back(a);
    }

    filesystem::create_directories(outdir);

    mt19937 rng(seed);

    for (const auto& path : files) {
        Instance I;
        if (!read_instance(path, I)) return 2;
        cout << "Instance: " << I.name << "   N=" << I.N << "  K=" << I.K << "\n";

        vector<MethodResult> methods;
        methods.push_back({"RANDOM", LLONG_MAX, {}, {}});
        methods.push_back({"NN_END_ONLY", LLONG_MAX, {}, {}});
        methods.push_back({"NN_INSERT_ANYWHERE_PATH", LLONG_MAX, {}, {}});
        methods.push_back({"GREEDY_CYCLE_CHEAPEST_INSERTION", LLONG_MAX, {}, {}});

        // RANDOM: 200 total (not per start)
        {
            auto &MR = methods[0];
            for (int r = 0; r < runs_per_start; ++r) {
                auto tour = random_solution(I, rng, -1);
                long long obj = objective(tour, I);
                MR.all_objs.push_back(obj);
                if (obj < MR.best_obj) { MR.best_obj = obj; MR.best_tour = tour; }
            }
        }

        // NN and greedy: 200 per start node
        for (int start = 0; start < I.N; ++start) {
            for (int r = 0; r < runs_per_start; ++r) {
                // NN_END_ONLY
                {
                    auto tour = nn_end_only(I, start, rng);
                    long long obj = objective(tour, I);
                    methods[1].all_objs.push_back(obj);
                    if (obj < methods[1].best_obj) { methods[1].best_obj = obj; methods[1].best_tour = tour; }
                }
                // NN_INSERT_ANYWHERE_PATH
                {
                    auto tour = nn_path_insert_anywhere(I, start, rng);
                    long long obj = objective(tour, I);
                    methods[2].all_objs.push_back(obj);
                    if (obj < methods[2].best_obj) { methods[2].best_obj = obj; methods[2].best_tour = tour; }
                }
                // GREEDY_CYCLE_CHEAPEST_INSERTION
                {
                    auto tour = greedy_cycle_cheapest_insertion(I, start, rng);
                    long long obj = objective(tour, I);
                    methods[3].all_objs.push_back(obj);
                    if (obj < methods[3].best_obj) { methods[3].best_obj = obj; methods[3].best_tour = tour; }
                }
            }
        }

        // summaries + best tours + SVGs
        string summary_csv = (filesystem::path(outdir) / (I.name + "_results_summary.csv")).string();
        ofstream S(summary_csv);
        S << "instance,method,runs,min,max,avg,best_obj\n";
        for (auto &MR : methods) {
            if (!MR.best_tour.empty()) {
                string why;
                bool ok = check_solution(MR.best_tour, I, &why);
                if (!ok) cerr << "WARNING: best tour for " << MR.method << " failed check: " << why << "\n";
            }
            long long mn = LLONG_MAX, mx = LLONG_MIN; long double sum = 0;
            for (auto v : MR.all_objs) { mn = min(mn, v); mx = max(mx, v); sum += v; }
            long double avg = (MR.all_objs.empty() ? 0 : sum / (long double)MR.all_objs.size());
            S << I.name << "," << MR.method << "," << MR.all_objs.size() << ","
              << mn << "," << mx << "," << (long long)(avg + 0.5) << "," << MR.best_obj << "\n";

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
    }
    return 0;
}
