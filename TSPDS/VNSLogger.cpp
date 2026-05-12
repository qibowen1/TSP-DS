#include "VNSLogger.h"
#include <cmath>
#include <map>

VNSLogger::VNSLogger(const TSPDSGraph& graph, const TSPDSAlgorithmParams& params)
    : graph(graph), params(params), logging_enabled(true) {
    // 生成默认日志文件名
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "vns_tspds_log_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".json";
    log_filename = ss.str();
}

VNSLogger::~VNSLogger() {
    if (log_file.is_open()) {
        finalizeLogging();
    }
}

// 获取节点类型
std::string VNSLogger::getNodeType(int node_id) const {
    if (node_id == graph.depot) return "depot";
    if (node_id == graph.drone_station) return "drone_station";
    if (!graph.is_drone_eligible[node_id] && graph.is_truck_only[node_id]) return "truck_only";
    if (graph.is_drone_eligible[node_id]) return "drone_eligible";
    return "regular";
}

// 添加图结构信息
void VNSLogger::addGraphStructureInfo() {
    if (!log_data.contains("graph_structure")) {
        log_data["graph_structure"] = json::object();
    }

    // 节点类型统计
    std::map<std::string, int> type_counts;
    for (int i = 0; i < graph.nodes.size(); ++i) {
        std::string type = getNodeType(i);
        type_counts[type]++;
    }

    log_data["graph_structure"]["node_type_counts"] = type_counts;
    log_data["graph_structure"]["total_nodes"] = graph.nodes.size();
    log_data["graph_structure"]["depot_id"] = graph.depot;
    log_data["graph_structure"]["drone_station_id"] = graph.drone_station;
}

// 添加节点坐标信息 - 这是核心功能
void VNSLogger::addNodeCoordinatesInfo() {
    if (!log_data.contains("graph_structure")) {
        log_data["graph_structure"] = json::object();
    }

    json nodes_array = json::array();
    for (int i = 0; i < graph.nodes.size(); ++i) {
        json node_info;
        node_info["id"] = i;
        node_info["x"] = graph.nodes[i].first;  // 记录x坐标
        node_info["y"] = graph.nodes[i].second;  // 记录y坐标
        //node_info["type"] = getNodeType(i);

        // 计算节点到关键位置的距离
        if (i != graph.depot) {

        }
        if (i != graph.drone_station) {
            node_info["time_to_drone_station"] = graph.truck_time[i][graph.drone_station];
            if (graph.is_drone_eligible[i]) {
                node_info["drone_time_to_station"] = graph.drone_time[i][graph.drone_station];
            }
        }

        nodes_array.push_back(node_info);
    }

    log_data["graph_structure"]["nodes"] = nodes_array;
}

// 添加距离矩阵信息
void VNSLogger::addDistanceMatricesInfo() {
    if (!log_data.contains("graph_structure")) {
        log_data["graph_structure"] = json::object();
    }

    // 记录距离矩阵的统计信息（避免记录整个大矩阵）
    json matrix_info;

    // 卡车时间矩阵统计
    double min_truck_time = std::numeric_limits<double>::max();
    double max_truck_time = 0.0;
    double total_truck_time = 0.0;
    int truck_time_count = 0;

    for (int i = 0; i < graph.truck_time.size(); ++i) {
        for (int j = 0; j < graph.truck_time[i].size(); ++j) {
            if (i != j) {
                min_truck_time = std::min(min_truck_time, graph.truck_time[i][j]);
                max_truck_time = std::max(max_truck_time, graph.truck_time[i][j]);
                total_truck_time += graph.truck_time[i][j];
                truck_time_count++;
            }
        }
    }

    matrix_info["truck_time"] = {
        {"min", min_truck_time},
        {"max", max_truck_time},
        {"average", total_truck_time / truck_time_count},
        {"symmetry_check", graph.truck_time.size() > 0 ?
            (std::abs(graph.truck_time[0][1] - graph.truck_time[1][0]) < 1e-6) : true}
    };

    // 无人机时间矩阵统计
    double min_drone_time = std::numeric_limits<double>::max();
    double max_drone_time = 0.0;
    double total_drone_time = 0.0;
    int drone_time_count = 0;

    for (int i = 0; i < graph.drone_time.size(); ++i) {
        for (int j = 0; j < graph.drone_time[i].size(); ++j) {
            if (i != j && graph.is_drone_eligible[i] && graph.is_drone_eligible[j]) {
                min_drone_time = std::min(min_drone_time, graph.drone_time[i][j]);
                max_drone_time = std::max(max_drone_time, graph.drone_time[i][j]);
                total_drone_time += graph.drone_time[i][j];
                drone_time_count++;
            }
        }
    }

    matrix_info["drone_time"] = {
        {"min", drone_time_count > 0 ? min_drone_time : 0.0},
        {"max", drone_time_count > 0 ? max_drone_time : 0.0},
        {"average", drone_time_count > 0 ? total_drone_time / drone_time_count : 0.0}
    };

    log_data["graph_structure"]["distance_matrices"] = matrix_info;
}

// 添加节点连接性信息
void VNSLogger::addNodeConnectivityInfo() {
    if (!log_data.contains("graph_structure")) {
        log_data["graph_structure"] = json::object();
    }

    // 计算每个节点的连接性指标
    json connectivity_info = json::array();

    for (int i = 0; i < graph.nodes.size(); ++i) {
        json node_conn;
        node_conn["node_id"] = i;
        node_conn["coordinates"] = {
            {"x", graph.nodes[i].first},
            {"y", graph.nodes[i].second}
        };
        node_conn["type"] = getNodeType(i);

        connectivity_info.push_back(node_conn);
    }

    log_data["graph_structure"]["node_connectivity"] = connectivity_info;
}

std::string VNSLogger::generateTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << milliseconds.count();
    return ss.str();
}

std::string VNSLogger::formatDuration(double seconds) {
    int hours = static_cast<int>(seconds) / 3600;
    int minutes = (static_cast<int>(seconds) % 3600) / 60;
    double secs = fmod(seconds, 60.0);

    std::stringstream ss;
    if (hours > 0) ss << hours << "h ";
    if (minutes > 0 || hours > 0) ss << minutes << "m ";
    ss << std::fixed << std::setprecision(3) << secs << "s";
    return ss.str();
}

bool VNSLogger::initializeLogging() {
    if (!logging_enabled) return false;

    log_file.open(log_filename);
    if (!log_file.is_open()) {
        std::cerr << "警告: 无法打开日志文件 " << log_filename << std::endl;
        logging_enabled = false;
        return false;
    }

    // 初始化日志数据结构
    log_data = json::object();
    log_data["metadata"] = {
        {"algorithm", "VNS_TSPDS"},
        {"timestamp", generateTimestamp()},
        {"version", "2.0"},  // 版本升级，支持图形数据
        {"log_format", "iteration_tracking_with_visualization"}
    };

    // 新增：添加完整的图结构信息
    addGraphStructureInfo();
    addNodeCoordinatesInfo();  // 核心：添加节点坐标信息
    addDistanceMatricesInfo();
    addNodeConnectivityInfo();

    log_data["problem_instance"] = {
        {"nodes_count", graph.nodes.size()},
        {"drone_count", graph.drone_count},
        {"depot", graph.depot},
        {"drone_station", graph.drone_station},
        {"truck_only_customers", graph.nodes.size() - std::count(graph.is_drone_eligible.begin(), graph.is_drone_eligible.end(), true)},
        {"drone_eligible_customers", std::count(graph.is_drone_eligible.begin(), graph.is_drone_eligible.end(), true)},
    };

    log_data["algorithm_parameters"] = {
        {"max_iterations", params.max_run_time},
        {"max_run_time", params.max_run_time},
        {"shaking_number", params.shaking_number},
        {"LS_number", params.LS_number},
        {"truck_convert_maxNode", params.truck_convert_maxNode},
        {"drone_convert_maxNode", params.drone_convert_maxNode},
        {"max_swap_attempts", params.max_swap_attempts},
        {"droneStation_maxMoveDistance", params.droneStation_maxMoveDistance}
    };

    log_data["performance_metrics"] = json::object();
    log_data["iterations"] = json::array();

    std::cout << "✓ 增强版日志记录器已初始化: " << log_filename << std::endl;
    std::cout << "✓ 图形可视化数据已包含 - 节点数: " << graph.nodes.size()
        << ", 无人机数: " << graph.drone_count << std::endl;
    return true;
}

void VNSLogger::logAlgorithmStart() {
    if (!logging_enabled || !log_file.is_open()) return;

    log_data["execution_info"] = {
        {"start_time", generateTimestamp()},
        {"status", "running"},
        {"visualization_support", true}  // 标记支持可视化
    };

    // 写入初始数据
    log_file << log_data.dump(4);
    log_file.flush();
}

void VNSLogger::logIterationData(int iteration, int k,
    const TSPDSSolution& currentSolution,
    const TSPDSSolution& bestSolution,
    double elapsed_seconds,
    bool accepted) {
    if (!logging_enabled || !log_file.is_open()) return;

    json iteration_log;
    iteration_log["iteration"] = iteration;
    iteration_log["k"] = k;
    iteration_log["elapsed_time"] = elapsed_seconds;
    iteration_log["elapsed_time_formatted"] = formatDuration(elapsed_seconds);
    iteration_log["accepted"] = accepted;
    iteration_log["timestamp"] = generateTimestamp();

    // 记录当前解信息
    logSolutionDetails(iteration_log, currentSolution, "current");

    // 记录最优解信息
    logSolutionDetails(iteration_log, bestSolution, "best");

    // 记录诊断和评分信息
    iteration_log["diagnosis"] = currentSolution.diagnosis;
    iteration_log["combined_score"] = currentSolution.combined_score;
    iteration_log["time_imbalance"] = currentSolution.time_imbalance;
    iteration_log["utilization_ratio"] = currentSolution.utilization_ratio;

    // 新增：添加可视化专用数据
    logSolutionVisualizationData(iteration_log, currentSolution);

    // 添加到迭代数组
    log_data["iterations"].push_back(iteration_log);

    // 实时写入文件
    log_file.seekp(0);
    log_file << log_data.dump(4);
    log_file.flush();
}

// 新增：记录解决方案可视化数据
void VNSLogger::logSolutionVisualizationData(json& iteration_log, const TSPDSSolution& solution) {
    json viz_data;

    // 1. 节点服务状态
    json node_states = json::array();
    for (int i = 0; i < graph.nodes.size(); ++i) {
        json node_state;
        node_state["id"] = i;
        node_state["served_by_truck"] = solution.served_by_truck[i];
        node_state["served_by_drone"] = solution.served_by_drone[i];

        node_states.push_back(node_state);
    }
    viz_data["node_states"] = node_states;

    // 2. 卡车路径可视化数据
    json truck_viz;
    truck_viz["route"] = solution.truck_route;

    // 计算路径段的几何信息
    json path_segments = json::array();
    for (size_t i = 0; i < solution.truck_route.size() - 1; ++i) {
        int from = solution.truck_route[i];
        int to = solution.truck_route[i + 1];

        json segment;
        segment["from"] = from;
        segment["to"] = to;
        segment["time"] = graph.truck_time[from][to];

        path_segments.push_back(segment);
    }
    truck_viz["path_segments"] = path_segments;
    viz_data["truck_visualization"] = truck_viz;

    // 3. 无人机任务可视化数据
    json drones_viz = json::array();
    for (const auto& [drone_id, tasks] : solution.drone_assignments) {
        json drone_viz;
        drone_viz["drone_id"] = drone_id;
        drone_viz["tasks"] = tasks;

        // 计算每个任务的几何信息
        json task_routes = json::array();
        for (int node : tasks) {
            json task_route;
            task_route["customer"] = node;
            task_route["drone_time"] = graph.drone_time[graph.drone_station][node];

            task_routes.push_back(task_route);
        }
        drone_viz["task_routes"] = task_routes;
        drones_viz.push_back(drone_viz);
    }
    viz_data["drones_visualization"] = drones_viz;

    // 4. 关键指标可视化
    viz_data["visualization_metrics"] = {
        {"truck_path_efficiency", solution.truck_completion_time / calculateRouteDistance(solution.truck_route, graph.truck_time)},
        {"drone_utilization_ratio", solution.utilization_ratio},
        {"balance_indicator", 1.0 - (std::abs(solution.truck_completion_time - solution.drone_completion_time) / solution.makespan)},
        {"solution_quality_score", 1.0 / solution.makespan}  // 越大越好
    };

    iteration_log["visualization_data"] = viz_data;
}

// 其余现有方法保持不变...
void VNSLogger::logSolutionDetails(json& iteration_log,
    const TSPDSSolution& solution,
    const std::string& prefix) {
    // 现有实现保持不变...
}

void VNSLogger::logTruckRoute(json& log_obj, const std::vector<int>& route) {
    // 现有实现保持不变...
}

void VNSLogger::logDroneAssignments(json& log_obj,
    const std::unordered_map<int, std::vector<int>>& assignments) {
    // 现有实现保持不变...
}

void VNSLogger::logAlgorithmEnd(int total_iterations, double total_time) {
    if (!logging_enabled || !log_file.is_open()) return;

    log_data["execution_info"]["end_time"] = generateTimestamp();
    log_data["execution_info"]["total_iterations"] = total_iterations;
    log_data["execution_info"]["total_time"] = total_time;
    log_data["execution_info"]["total_time_formatted"] = formatDuration(total_time);
    log_data["execution_info"]["status"] = "completed";

    finalizeLogging();
}

void VNSLogger::finalizeLogging() {
    if (!log_file.is_open()) return;

    log_file.seekp(0);
    log_file << log_data.dump(4);
    log_file.close();

    std::cout << "✓ 增强版日志记录已完成: " << log_filename
        << " (记录 " << getLoggedIterations() << " 次迭代)" << std::endl;
    std::cout << "✓ 可视化数据已包含 - 可用于图形化展示解决方案" << std::endl;
}

// 静态工具方法
json VNSLogger::solutionToJson(const TSPDSSolution& solution, const TSPDSGraph& graph) {
    json solution_json;
    solution_json["makespan"] = solution.makespan;
    solution_json["truck_completion_time"] = solution.truck_completion_time;
    solution_json["drone_completion_time"] = solution.drone_completion_time;
    solution_json["station_activation_time"] = solution.station_activation_time;
    solution_json["combined_score"] = solution.combined_score;
    solution_json["diagnosis"] = solution.diagnosis;
    return solution_json;
}

double VNSLogger::calculateRouteDistance(const std::vector<int>& route,
    const std::vector<std::vector<double>>& time_matrix) {
    if (route.size() < 2) return 0.0;

    double total_distance = 0.0;
    for (size_t i = 0; i < route.size() - 1; ++i) {
        total_distance += time_matrix[route[i]][route[i + 1]];
    }
    return total_distance;
}

// setter 方法实现
void VNSLogger::setLoggingEnabled(bool enabled) {
    logging_enabled = enabled;
    if (!enabled && log_file.is_open()) {
        log_file.close();
    }
}

void VNSLogger::setLogFilename(const std::string& filename) {
    log_filename = filename;
}

void VNSLogger::setGraph(const TSPDSGraph& new_graph) {
    graph = new_graph;
}

void VNSLogger::setParams(const TSPDSAlgorithmParams& new_params) {
    params = new_params;
}