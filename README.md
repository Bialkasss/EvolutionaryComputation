## Compile:
g++ -std=c++17 -O3 -o EC_algorithms.exe main.cpp core/Instance.cpp core/Objective.cpp heuristics/Greedy.cpp heuristics/Regret.cpp heuristics/LocalSearch.cpp heuristics/LocalSearchCandidate.cpp heuristics/ListMoves.cpp heuristics/MultipleStartLS_IterativeLS.cpp heuristics/LargeNeighbourhoodSearch.cpp heuristics/HybridEvolutionary.cpp   -Icore -Iutils -Iheuristics -lstdc++fs

## Run:
 ./EC_algorithms.exe ../Data/TSPA.csv ../Data/TSPB.csv