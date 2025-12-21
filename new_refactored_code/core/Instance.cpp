#include "Instance.h"

// Include all the necessary standard libraries
// that these functions use.
#include <cmath>
#include <cctype>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <numeric>
#include <algorithm>

//
// These are the *implementations* of the helper functions.
// We've copied them directly from main.cpp.
// By making them 'static', we hide them inside this .cpp file,
// since only read_instance() needs them.
//

static inline int iround(double v) {
    return (int) llround(v);
}

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


//
// This is the *implementation* of the main read_instance function.
// The code is copied directly from your main.cpp file.
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


#include <numeric>
#include <algorithm>

// ... (rest of the file is the same until the end of read_instance) ...

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

    // Compute nearest neighbors for each node
    inst.nearest_neighbors.assign(inst.N, vector<int>());
    for (int i = 0; i < inst.N; ++i) {
        vector<int> p(inst.N);
        iota(p.begin(), p.end(), 0); // Fills p with 0, 1, 2, ..., N-1
        sort(p.begin(), p.end(), [&](int a, int b) {
            return inst.D[i][a] < inst.D[i][b];
        });
        // We don't need to include the node itself in its neighbors list
        for (int node_idx : p) {
            if (node_idx != i) {
                inst.nearest_neighbors[i].push_back(node_idx);
            }
        }
    }

    return true;
}

