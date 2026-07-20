#pragma once
// ���ඨ�������Ӳ����ṹ��
struct TSPDSAlgorithmParams {

    bool verbose = true;

	int drone_node_k_number = 60; 
    bool initial_backbone_seed = true;
    bool initial_target_seed = false;
    int initial_target_seed_count = 1;
    int target_seed_random_top = 1;
    bool initial_paper_seed = false;
    int backbone_insert_candidates = 80;

	int max_run_time = 1200; // ��λ����

    double golbal_attemp_per_truck = 0.4;
    double golbal_attemp_per_drone = 0.4;

    int population_size = 10;//��Ⱥ��С
    int population_rebuild_threshold = 5;//�����Ⱥ������������ô���
    bool stochastic_neighborhoods = false;
    int final_vnd_passes = 1;
    int local_search_no_improve_limit = 1;//�ڲ�vnd���������ֹͣ����
    double mutation_probability = 0.15;//�������
    bool route_perturbation_enabled = false;
    int mutation_k = 1;//�Ŷ���ģ
    bool elite_perturbation_enabled = true;
    int elite_perturbation_max_k = 8;
    bool lkh_after_local_search = false;
    int lkh_runs = 1;
    int final_lkh_runs = 1;
    bool final_full_lkh = false;
    bool vnd_perturb_only = false;
    bool compound_exchange_enabled = true;
    int compound_top_truck = 20;
    int compound_top_drone = 20;
    int compound_pair_top = 8;
    bool compound_lkh_refine = false;
    bool route_oropt_enabled = false;
    bool crossover_enabled = true;
    bool ls_two_opt_enabled = true;
    bool ls_cross_station_enabled = true;
    bool ls_drone_balance_enabled = true;
    bool ls_truck_bottleneck_enabled = true;
    bool ls_drone_bottleneck_enabled = true;
    bool ls_truck_drone_swap_enabled = true;
    bool ls_farthest_truck_to_drone_enabled = true;
    bool target_assignment_enabled = false;
    int target_assignment_top_truck = 20;
    int target_assignment_top_drone = 20;
    int target_assignment_max_children = 6;
    bool target_vnd_enabled = false;
    bool target_ladder_enabled = true;
    double target_value = 0.0;
    int target_vnd_max_passes = 30;
    int target_vnd_top_truck = 30;
    int target_vnd_top_drone = 30;

    int balance_topK_nodes = 40;
    int drone_balance_maxIter = 100;

};
