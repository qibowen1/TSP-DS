#pragma once
#include "TSPDSGraph.h"
#include "TSPDSSolution.h"
#include "TSPDS_Params.h"
#include <iostream>
#include <fstream>
#include "json.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <numeric>

using json = nlohmann::json;

class VNSLogger {
private:
    std::ofstream log_file;
    json log_data;
    std::string log_filename;
    bool logging_enabled;
    TSPDSGraph graph;
    TSPDSAlgorithmParams params;

    // 内部辅助方法
    std::string generateTimestamp();
    std::string formatDuration(double seconds);
    std::string getNodeType(int node_id) const;

    // 新增的图结构信息记录方法
    void addGraphStructureInfo();
    void addNodeCoordinatesInfo();
    void addDistanceMatricesInfo();
    void addNodeConnectivityInfo();

public:
    VNSLogger(const TSPDSGraph& graph, const TSPDSAlgorithmParams& params);
    ~VNSLogger();

    // 配置方法
    void setLoggingEnabled(bool enabled);
    void setLogFilename(const std::string& filename);
    void setGraph(const TSPDSGraph& new_graph);
    void setParams(const TSPDSAlgorithmParams& new_params);

    // 主要日志记录方法
    bool initializeLogging();
    void logAlgorithmStart();
    void logIterationData(int iteration, int k,
        const TSPDSSolution& currentSolution,
        const TSPDSSolution& bestSolution,
        double elapsed_seconds,
        bool accepted);
    void logSolutionDetails(json& iteration_log,
        const TSPDSSolution& solution,
        const std::string& prefix);
    void logTruckRoute(json& log_obj, const std::vector<int>& route);
    void logDroneAssignments(json& log_obj,
        const std::unordered_map<int, std::vector<int>>& assignments);
    void logAlgorithmEnd(int total_iterations, double total_time);
    void finalizeLogging();

    // 新增的可视化数据记录方法
    void logSolutionVisualizationData(json& iteration_log, const TSPDSSolution& solution);

    // 状态查询方法
    bool isLoggingEnabled() const { return logging_enabled; }
    bool isFileOpen() const { return log_file.is_open(); }
    std::string getLogFilename() const { return log_filename; }
    size_t getLoggedIterations() const {
        return log_data.contains("iterations") ? log_data["iterations"].size() : 0;
    }

    // 工具方法
    static json solutionToJson(const TSPDSSolution& solution,
        const TSPDSGraph& graph);
    static double calculateRouteDistance(const std::vector<int>& route,
        const std::vector<std::vector<double>>& time_matrix);
};