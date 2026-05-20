# Large-Scale Travelling Salesman Problem with Drone Station: Approximation and Heuristic Algorithms
The software and data in this repository are a snapshot of the software and data that were used in the research reported on in the paper.

# Description
Truck-drone coordination has emerged as a promising solution to meet the increasing demand of last-mile delivery.
In this paper, we study the travelling salesman problem with a drone station (TSP-DS).
In this problem, a truck transports goods to a drone station, after which truck and drones serve customers in parallel. The objective is to minimize the delivery makespan. 
Despite its practical relevance, TSP-DS is computationally challenging and existing solution methods are limited to small-scale instances or yield inferior performance.
In this paper, we first propose a fast approximation algorithm, named Appro. Based on a decomposition analysis. Appro guarantees a theoretical approximation ratio of 12. We then develop a multi-neighbourhood search algorithm, MNS, to further improve solution quality. MNS employs multiple complementary neighbourhood structures within a variable neighbourhood descent framework. We also introduce efficient implementation techniques to accelerate the search. Extensive computational experiments on 222 benchmark instances demonstrate the effectiveness of the proposed methods. 
MNS consistently outperforms existing approaches in solution quality, and it effectively provides solutions for instances with more than 1,000 customer nodes, a task at which all other known methods exhibit substantially inferior performance or even fail.  Meanwhile, Appro quickly produces feasible solutions with a provable approximation guarantee. 

<p align="center">
  <img src="https://github.com/qibowen1/TSP-DS/blob/main/TSPDS%20problem.png?raw=true"  alt="Sublime's custom image" />
</p>

This project contains three folders which contain three algorithms: MNS (TSPDS folder), Appro (APPROXIMATE folder) and TPLS (MTSPDS folder).

# Prerequisites
The codes are implemented under Ubuntu 22.04. Boost C++ 17 is also required for running the codes.

# Build and Run MNS
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

# Build and Run Appro
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

