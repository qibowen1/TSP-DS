#pragma once
// 在类定义中添加参数结构体
struct TSPDSAlgorithmParams {

    bool verbose = true;

	int drone_node_k_number = 60; 

	int max_run_time = 1200; // 单位：秒

    double golbal_attemp_per_truck = 0.4;
    double golbal_attemp_per_drone = 0.4;

    int population_size = 10;//种群大小
    int population_rebuild_threshold = 5;//外层种群最大无提升重置次数
    int local_search_no_improve_limit = 30;//内层vnd最大无提升停止次数
    double mutation_probability = 0.15;//变异概率
    int mutation_k = 1;//扰动规模

    int balance_topK_nodes = 40;
    int drone_balance_maxIter = 100;

};
