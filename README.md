# TSP-DS

```markdown
## Build and Run

In Linux operating system, enter the project directory, put the dataset into TSP-DS/TSPDS/data/tsp_origan, and run the program as follows:

```bash
# to run TSP-DS
cd TSP-DS/TSPDS

rm -rf build

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCPLEX_ROOT=/path/to/CPLEX_Studio201 #your cplex system path

cmake --build build -j$(nproc)

./build/bin/TSPDS.out

#to run TPLS
cd TSP-DS/MTSPDS

rm -rf build

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCPLEX_ROOT=/path/to/CPLEX_Studio201

cmake --build build -j$(nproc)

./build/bin/MTSPDS.out


