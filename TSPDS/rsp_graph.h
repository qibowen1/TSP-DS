#pragma once
#include <vector>
#include <unordered_map>
#include <utility>
#include <limits>

struct RSPGraph {
    std::vector<std::pair<double, double>> nodes;
    std::vector<std::vector<double>> routing_cost;
    std::vector<std::vector<double>> assign_cost;
    int depot = 0;

    void initialize(int size) {
        routing_cost.resize(size, std::vector<double>(size));
        assign_cost.resize(size, std::vector<double>(size));
    }
};

struct Solution {
    std::vector<int> ring;
    std::unordered_map<int, int> assignments;
    // 新增：记录每个环节点分配的非环节点
    std::unordered_map<int, std::vector<int>> assignment_map;
    std::vector<bool> in_ring;
    double routing_cost = 0;
    double assign_cost = 0;


    double total_cost() const { return routing_cost + assign_cost; }
    void initialize(int size) {
        in_ring.resize(size, false);
        ring.push_back(0);
        in_ring[0] = true;
        // 初始化所有非depot节点分配到depot
        for (int u = 1; u < size; ++u) {
            assignments[u] = 0;
            assignment_map[0].push_back(u);
        }
    }
};

// 评估每个环节点的重要性评分
struct NodeImportance {
    int node_id;
    double routing_contribution;  // 对环成本的贡献
    double assignment_benefit;    // 对分配成本的贡献
    double importance_score;       // 综合重要性
};

