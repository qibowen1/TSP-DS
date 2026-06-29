#pragma once
#include <vector>
#include <utility>
#include <limits>
#include <cmath>
#include <algorithm>

struct MTSPDSGraph {
    using Coord = std::pair<double, double>;
    static constexpr double INF = std::numeric_limits<double>::infinity();

    // geometry
    std::vector<Coord> nodes;

    // travel time matrices
    std::vector<std::vector<double>> truck_time; // Manhattan (int-rounded)
    std::vector<std::vector<double>> drone_time; // Euclidean (int-rounded) * speed_ratio

    int depot = 0;

    // node types
    std::vector<bool> is_station;     // candidate station nodes
    std::vector<bool> is_customer;    // customer nodes
    std::vector<bool> is_truck_only;  // customer that cannot be served by drones

    // sets
    std::vector<int> stations;   // indices where is_station=true
    std::vector<int> customers;  // indices where is_customer=true

    // problem parameters (paper notation)
    int KN = 2;        // number of trucks
    int C = 4;        // max activated stations
    int DN = 1;        // drones per station
    double alpha = 2;  // relative drone speed factor in paper (we map via speed_ratio)
    double max_roundtrip_dist = INF; // E in paper (distance bound for drone round trip)

    // in your original code you used speed_ratio = truck_speed / drone_speed (smaller => drone faster)
    double speed_ratio = 0.5;

    // TSPLIB nint
    static inline int nint(double x) { return (int)std::floor(x + 0.5); }

    static inline int euclidInt(const Coord& a, const Coord& b) {
        double dx = a.first - b.first, dy = a.second - b.second;
        return nint(std::sqrt(dx * dx + dy * dy));
    }
    static inline int manhattanInt(const Coord& a, const Coord& b) {
        return nint(std::abs(a.first - b.first) + std::abs(a.second - b.second));
    }

    void initialize(int n) {
        nodes.resize(n);
        truck_time.assign(n, std::vector<double>(n, 0));
        drone_time.assign(n, std::vector<double>(n, 0));
        is_station.assign(n, false);
        is_customer.assign(n, false);
        is_truck_only.assign(n, false);
        stations.clear(); customers.clear();
    }

    void rebuildSets() {
        stations.clear(); customers.clear();
        for (int i = 0;i < (int)nodes.size();++i) {
            if (i == depot) continue;
            if (is_station[i]) stations.push_back(i);
            else if (is_customer[i]) customers.push_back(i);
        }
    }

    void initDistanceMatrices() {
        int n = (int)nodes.size();
        truck_time.assign(n, std::vector<double>(n, 0));
        drone_time.assign(n, std::vector<double>(n, 0));
        for (int i = 0;i < n;++i) {
            for (int j = 0;j < n;++j) {
                int td = manhattanInt(nodes[i], nodes[j]);
                int dd = euclidInt(nodes[i], nodes[j]);
                truck_time[i][j] = (double)td;
                drone_time[i][j] = (double)dd * speed_ratio;
            }
        }
    }

    // eligibility: round-trip distance <= E (paper constraint (11) uses distance traveled E)
    bool droneEligible(int s, int cust) const {
        if (is_truck_only[cust]) return false;
        if (!is_station[s]) return false;
        if (!is_customer[cust]) return false;
        int d = euclidInt(nodes[s], nodes[cust]);
        return (2.0 * (double)d) <= max_roundtrip_dist;
    }
};
