Input format:
```
N q m s
customer_1 customer_2 ... customer_q
g
truck_only_1 ... truck_only_g
d
drone_eligible_1 ... drone_eligible_d
truck_time_matrix N x N
drone_time_matrix N x N
```
Node meaning:
- depot is node 0
- drone station is node s
- C is the customer set
- Cg is the truck-only customer set
- Cd is the drone-eligible customer set
- tg[i][j] is truck travel time
- td[i][j] is one-way drone travel time from i to j
- each drone delivery is a round trip, so its processing time is 2 * td[s][j]

Compile:
```
g++ -std=c++17 -O2 appro.cpp -o appro
```

Run:
```
./appro ./data/[dataset name].csv [#drones] [truck travel time/per mile, 1 by default] [drone travel time/per mile, 0.5 by default] [result file name].csv
```
