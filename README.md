# Large-Scale Travelling Salesman Problem with Drone Station: Approximation and Heuristic Algorithms
The software and data in this repository are a snapshot of the software and data that were used in the research reported on in the paper.

# Description
Truck-drone coordination has emerged as a promising solution to meet the increasing demand of last-mile delivery.
We study the travelling salesman problem with a drone station (TSP-DS), where a truck transports goods to a drone station, after which the truck and drones serve customers in parallel, and the objective is to minimize the delivery makespan. Such settings generalize the parallel drone scheduling travelling salesman problem (PDSTSP). They arise in delivery systems where drone infrastructure has already been deployed and daily operational decisions require coordinating truck routing and drone scheduling.

To solve TSP-DS, we propose two algorithms.
We first propose a memetic search algorithm, TD-Heur, that consists of a novel bottleneck-guided local search, a novel component-based crossover operator and a population management mechanism tailored to TSP-DS.
Theoretically, we further show that TSP-DS can be decomposed into two well-studied subproblems with provable guarantees. Based on the decomposition, we develop the first approximation algorithm, TD-Appro, with an approximation ratio of $5$.

Computational experiments demonstrate that TD-Appro quickly produces feasible solutions with quality guarantees. 
Meanwhile, TD-Heur achieves the best known value on 164 instances and obtains new upper bounds on 50 instances.

<p align="center">
  <img src="https://github.com/qibowen1/TSP-DS/blob/main/TSPDS%20problem.png?raw=true"  alt="Sublime's custom image" />
</p>

This project contains three folders which contain three algorithms: TD-Heur (TSPDS folder), TD-Appro (APPROXIMATE folder) and TPLS (MTSPDS folder).

# Prerequisites
The codes are implemented under Ubuntu 22.04. Boost C++ 17 is also required for running the codes.

# Build and Run TD-Heur
In Linux operating system, enter the project directory, put the dataset into TSP-DS/TSPDS/data/tsp_origan, and run the program as follows:
```
cd TSP-DS/TSPDS

rm -rf build

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release

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
cd TSP-DS/TPLS

rm -rf build

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build -j$(nproc)

./build/bin/MTSPDS.out
```
