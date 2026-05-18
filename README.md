# TSP-DS

```markdown
## Build and Run

In Linux/Ubuntu operating system, enter the executable directory and run the program as follows:

```bash

# Enter the project directory
# data放在TSP-DS-main/TSPDS/data/tsp_origan 下
#RUN TSP-DS
cd TSP-DS-main/TSPDS

# Configure and build
rm -rf build

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCPLEX_ROOT=/path/to/CPLEX_Studio201 #cplex路径

cmake --build build -j$(nproc)

./build/bin/TSPDS.out

#RUN MTSPDS
# Enter the project directory
cd MTSPDS

# Configure and build
rm -rf build

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCPLEX_ROOT=/path/to/CPLEX_Studio201

cmake --build build -j$(nproc)

./build/bin/MTSPDS.out

#RUN APPROXIMATE
Compile:
```
g++ -std=c++17 -O2 appro.cpp -o appro
```

Run:
```
./appro ./data/[dataset name].csv [#drones] [truck travel time/per mile, 1 by default] [drone travel time/per mile, 0.5 by default] [result file name].csv
```

#data url 通过网盘分享的文件：data.zip链接: https://pan.baidu.com/s/1I21oIH5Q6w6E5WzYzk1TZA?pwd=stjj 提取码: stjj

