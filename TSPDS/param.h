#pragma once
// 在类定义中添加参数结构体
struct AlgorithmParams {
	//构造阶段参数
    double construction_rcl = 0.5;    // 优化构造阶段
    int max_stagnation = 1; // 允许的最大停滞次数


	//vns参数
    double max_time = 180;  // 60秒时间限制
    int k_max = 50;           // 扰动强度
	int shaking_size = 4;    // 扰动算子数量
	int local_search_size = 6; // 局部搜索算子数量
	int max_iterVNS = 200; // VNS最大迭代次数

	// 模拟退火参数
	int initial_temperature = 1000; // 初始温度
    double final_temperature = 1.0;        // 终止温度
    double cooling_rate = 0.90;            // 冷却系数
    int sa_iterations_per_temp = 5;       // 每个温度的迭代次数
    bool enable_simulated_annealing = true; // 启用SA接受准则


    // 算子参数
	int min_exchange_size = 15; // exchange 最小交换节点数
	int NEIGHBORHOOD_SIZE = 15; // exchange 每个环节点考虑的非环节点数量

	int batch_intelligent_removal_rito = 0.1; // 批量智能删除节点数

    // 新参数
    double cluster_distance_threshold = 100.0; // 簇距离阈值
    int max_cluster_size = 5;                 // 最大簇大小
    int min_cluster_size = 2;                 // 最小簇大小
    int centrality_exchange_trials = 20;     // 中心性交换尝试次数
    int segment_length = 4;                   // 路径片段长度
    int max_critical_nodes = 10;              // 最大关键节点数


    // 禁忌搜索优化
     int DEFAULT_TABU_TENURE = 10;            // 默认禁忌期限
     int MAX_TABU_SIZE = 30;                // 禁忌表最大大小

};
