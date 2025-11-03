g++ -std=c++17 -O3 -o EC_algorithms.exe main.cpp core/Instance.cpp core/Objective.cpp heuristics/Greedy.cpp heuristics/Regret.cpp heuristics/LocalSearch.cpp -Icore -Iutils -Iheuristics -lstdc++fs

 ./EC_algorithms.exe ../Data/TSPA.csv ../Data/TSPB.csv