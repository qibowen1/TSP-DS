Input format: a csv file
```
- column 1: node ID
- column 2: x-coordinate of node
- column 3: y-coordinate of node
- column 4: customer type,  0: drone-eligible customer; 1: truck-only customer;2: drone station.
- both the first node and last node represent the same depot node
```

Output format: a csv file
```
- Customers:  number of customers
- Drone Eligible Customers:  number of drone eligible customers
- Truck Only Customers	number of truck-only customers
- Truck Time Factor: truck travel time per mile
- Drone Time Factor: drone travel time per mile
- Truck Tour Cost: truck makespan
- Drone Activation Time
- Drone Makespan
- Makespan: delivery makespan
- PCPath
- TruckTour
- Truck Served Customers
- Drone Served Customers	
- Drone Schedules
- Runtime in seconds	
```

Compile:
```
g++ -std=c++17 -O2 appro.cpp -o appro
```

Run:
```
./appro ./data/[dataset name].csv [#drones] [truck travel time/per mile, 1 by default] [drone travel time/per mile, 0.5 by default] [result file name].csv
```
