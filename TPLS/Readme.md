# Build and Run TPLS
In Linux operating system, enter the project directory, put the dataset into TSP-DS/TPLS/data/tsp_origan, and run the program as follows:
```
cd TSP-DS/TPLS

rm -rf build

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build -j$(nproc)

./build/bin/MTSPDS.out
```
