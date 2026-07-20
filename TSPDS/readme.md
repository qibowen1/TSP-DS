# Build and Run TD-Heur

In Linux operating system, enter the project directory, put the dataset into TSP-DS/TSPDS/data/tsp_origan, and run the program as follows:

```bash
cd TSP-DS/TSPDS

rm -rf build

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

cmake --build build -j$(nproc)

./build/bin/TSPDS.out
```
