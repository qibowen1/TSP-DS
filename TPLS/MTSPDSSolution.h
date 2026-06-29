#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <limits>

struct StationSchedule {
    double makespan = 0.0;                  // max load among DN drones
    std::vector<std::vector<int>> tasks;    // tasks[d] = list of customers
    std::vector<double> load;               // load[d]
};

struct MTSPDSSolution {
    static constexpr double INF = std::numeric_limits<double>::infinity();

    // K truck tours, each is depot ... depot
    std::vector<std::vector<int>> truck_routes; // size KN

    // customer -> station (if served by drones), else -1
    std::vector<int> customer_station; // size n, only meaningful for customers
    // customer -> truck (if served by trucks), else -1
    std::vector<int> customer_truck;

    // station -> schedule
    std::unordered_map<int, StationSchedule> station_schedule;

    // derived times
    std::vector<double> truck_completion;                 // per truck
    std::unordered_map<int, double> station_activation;    // station -> arrival time of the truck visiting it
    std::unordered_map<int, double> station_completion;    // station -> activation + schedule.makespan
    double makespan = INF;

    void initialize(int n, int KN) {
        truck_routes.assign(KN, {});
        customer_station.assign(n, -1);
        customer_truck.assign(n, -1);
        station_schedule.clear();
        truck_completion.assign(KN, 0.0);
        station_activation.clear();
        station_completion.clear();
        makespan = INF;
    }
};
