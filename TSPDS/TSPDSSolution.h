#pragma once
#include <string>
#include <unordered_map>

struct TSPDSSolution {
    int max_resAt = 0; //记录最后一次改进的迭代次数
	int find_best_time = 0; //记录找到最优解所用的时间
    int dorne_visit_dep = 0;
    int total_iter = 0;
    int init_Id = 0;

    std::vector<int> pos_in_truck; //卡车节点位置映射
    int pos_station_in_truck = -1;     // station 在 truck_route 中的位置（缓存）

    // 卡车路径（必须包含depot和无人机站）
    std::vector<int> truck_route;

    // 无人机任务分配
    std::unordered_map<int, std::vector<int>> drone_assignments; // 无人机id->节点列表
    std::unordered_map<int, int> node_to_drone;                  // 节点->无人机映射

    // 服务方式标记
    std::vector<bool> served_by_truck;    // 节点是否由卡车服务
    std::vector<bool> served_by_drone;    // 节点是否由无人机服务

    // 时间计算
    double truck_completion_time = 0;     // 卡车完成时间
    double drone_completion_time = 0;     // 无人机完成时间 (已经加了激活时间的)
    double makespan = 0;                  // 总完成时间 = max(卡车时间, 无人机站激活时间+无人机时间)

    // 无人机站激活时间
    double station_activation_time = 0;    // 卡车到达无人机站的时间
    int drone_Id = 0; //无人机站编号
    int drone_total_task = 0;
    // 新增：综合评估和诊断信息
    double combined_score;
    double time_imbalance; // 时间不平衡度
    double utilization_ratio; // 资源利用率
    std::string diagnosis; // 问题诊断信息

    void initialize(int size) {
        served_by_truck.resize(size, false); 
        served_by_drone.resize(size, false);
        pos_in_truck.assign(size, -1);
        pos_station_in_truck = -1;
        combined_score = std::numeric_limits<double>::max();
        time_imbalance = 0.0;
        utilization_ratio = 0.0;
        diagnosis = "";
    }

    // 新的目标函数计算
    double total_time() const {
        return makespan;
    }
};
