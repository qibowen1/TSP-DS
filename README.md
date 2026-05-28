# Large-Scale Travelling Salesman Problem with Drone Station: Approximation and Heuristic Algorithms
The software and data in this repository are a snapshot of the software and data that were used in the research reported on in the paper.

# Description
Truck-drone coordination has emerged as a promising solution to meet the increasing demand of last-mile delivery.
In this paper, we study the traveling salesman problem with a drone station (TSP-DS).
In this problem, a truck transports goods to a drone station, after which the truck and drones serve customers in parallel, and the objective is to minimize the delivery makespan. 
Despite its practical relevance, TSP-DS is computationally challenging and existing solution methods are limited to small-scale instances or yield inferior performance.
For the first time, we propose a fast polynomial-time algorithm, \textit{TD-Appro}, with a 12-approximation ratio. The key idea is to decouple TSP-DS into two well-studied problems, both of which have efficient algorithms.
We further develop a local search-based heuristic algorithm, \textit{TD-Heur}, to further improve solution quality. \textit{TD-Heur} employs multiple complementary neighbourhood structures within a variable neighbourhood descent framework. In addition, we devise efficient implementation techniques to accelerate the search. 
Extensive computational experiments on 222 benchmark instances demonstrate the effectiveness of the proposed methods.
\textit{TD-Appro} quickly produces feasible solutions. 
Meanwhile,  \textit{TD-Heur} consistently outperforms existing approaches in solution quality. 

<p align="center">
  <img src="https://github.com/qibowen1/TSP-DS/blob/main/TSPDS%20problem.png?raw=true"  alt="Sublime's custom image" />
</p>

This project contains three folders which contain three algorithms: MNS (TSPDS folder), Appro (APPROXIMATE folder) and TPLS (MTSPDS folder).

# Prerequisites
The codes are implemented under Ubuntu 22.04. Boost C++ 17 is also required for running the codes.

# Build and Run TD-Heur
In Linux operating system, enter the project directory, put the dataset into TSP-DS/TSPDS/data/tsp_origan, and run the program as follows:
```
cd TSP-DS/TSPDS

rm -rf build

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCPLEX_ROOT=/path/to/CPLEX_Studio1201 #your cplex system path the default location is /opt/ibm/ILOG/CPLEX_Studio1210

cmake --build build -j$(nproc)

./build/bin/TSPDS.out
```

# Build and Run TD-Appro
Compile:
```
cd TSP-DS/APPROXIMATE

g++ -std=c++17 -O2 appro.cpp -o appro

./appro ./data/[dataset name].csv [#drones] [truck travel time/per mile, 1 by default] [drone travel time/per mile, 0.5 by default] [result file name].csv
```

# Build and Run TPLS
In Linux operating system, enter the project directory, put the dataset into TSP-DS/MTSPDS/data/tsp_origan, and run the program as follows:
```
cd TSP-DS/MTSPDS

rm -rf build

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCPLEX_ROOT=/path/to/CPLEX_Studio1201

cmake --build build -j$(nproc)

./build/bin/MTSPDS.out
```

