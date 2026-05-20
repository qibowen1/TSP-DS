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
