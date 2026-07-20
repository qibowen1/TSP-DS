#pragma once
#include <vector>
#include <unordered_map>
#include <utility>
#include <limits>
#include <random>
#include <cmath> // std::abs
using namespace std;
const double INF = numeric_limits<double>::max(); // 定义无穷大常量

struct TSPDSGraph {
    std::vector<std::pair<double, double>> nodes;

    // 时间矩阵替代成本矩阵
    std::vector<std::vector<double>> truck_time;    // 卡车旅行时间
    std::vector<std::vector<double>> drone_time;    // 无人机旅行时间

    // 特殊节点标识
    int depot = 0;
    int drone_station = -1;

    // 节点类型分类
    std::vector<bool> is_truck_only;      // 只能由卡车服务的节点
    std::vector<bool> is_drone_eligible; // 可由无人机服务的节点
    std::vector<bool> is_drone_station;   // 无人机站标记

    // 无人机相关参数
    int drone_count = 1;                  // 无人机数量
    double drone_range = INF;             // 无人机飞行范围
    double speed_ratio = 0.5;             // 卡车速度:无人机速度比例

    void initialize(int size) {
        truck_time.resize(size, std::vector<double>(size));
        drone_time.resize(size, std::vector<double>(size));
        is_truck_only.resize(size, false);
        is_drone_eligible.resize(size, false);
        is_drone_station.resize(size, false);
    }

    // TSPLIB 常用的 nint：四舍五入到最近整数
    static inline int nint(double x) {
        return static_cast<int>(std::floor(x + 0.5));
    }

    // 计算欧几里得距离（取整）
    static int euclideanDistanceInt(const std::pair<double, double>& a,
        const std::pair<double, double>& b) {
        double dx = a.first - b.first;
        double dy = a.second - b.second;
        return nint(std::sqrt(dx * dx + dy * dy));
    }

    // 计算曼哈顿距离（取整）
    static int manhattanDistanceInt(const std::pair<double, double>& a,
        const std::pair<double, double>& b) {
        double v = std::abs(a.first - b.first) + std::abs(a.second - b.second);
        return nint(v);
    }

    static double euclideanDistance(const std::pair<double, double>& a,
        const std::pair<double, double>& b) {
        double dx = a.first - b.first;
        double dy = a.second - b.second;
        return std::sqrt(dx * dx + dy * dy);
    }

    // 计算曼哈顿距离
    static double manhattanDistance(const std::pair<double, double>& a,
        const std::pair<double, double>& b) {
        double v = std::abs(a.first - b.first) + std::abs(a.second - b.second);
        return v;
    }

    // 初始化基于坐标的距离矩阵
    void initDistanceMatrices(const vector<pair<double, double>>& coords, double speed_ratio) {
        int n = coords.size();
        truck_time.resize(n, vector<double>(n));
        drone_time.resize(n, vector<double>(n));
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                truck_time[i][j] = manhattanDistance(coords[i], coords[j]);
                drone_time[i][j] = euclideanDistance(coords[i], coords[j]) * speed_ratio;//对的 无人机速度快，时间段
            }
        }
    }

};

