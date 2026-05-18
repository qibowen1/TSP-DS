#pragma once
// 在类定义中添加参数结构体
struct TSPDSAlgorithmParams {


    bool verbose = true;

	int drone_node_k_number = 60; //初始离无人机站最近的节点数量

	int common_size = 100;

	int max_run_time = 1200; // 单位：秒

    double golbal_attemp_per_truck = 0.4;
    double golbal_attemp_per_drone = 0.4;
	double golbal_attemp_adjust_drone_station = 0.3;

    int restart_no_improve_iters = 1;

    //shaking数量
    int shaking_number = 10;

    int balance_topK_nodes = 40;
    int drone_balance_maxIter = 100;
    int truck_block_maxNode = 50;
    //LS
    int LS_number = 1;

    //无人机
    //卡车相关
    int truck_convert_maxNode = common_size;


    //无人机站
    int drone_convert_maxNode = common_size;
	int droneStation_maxMoveDistance = common_size;

    //交换节点
    int max_swap_attempts = common_size;


};
