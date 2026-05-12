#include "solver.h"
#include <algorithm>
#include <random>
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <queue>
#include <numeric>
#include <limits>
#include <unordered_set>  // 必须包含该头文件
#include <fstream>
#include <sstream>
#include <string>
#include <filesystem>
#include <iomanip>
#include <ctime>
#include <functional>

using namespace std;
// 在需要使用shuffle的函数内部定义
std::random_device rd;  // 随机种子
std::mt19937 rng(rd()); // 使用Mersenne Twister引擎
const double INF = numeric_limits<double>::max(); // 定义无穷大常量

// LKH库函数声明
#ifdef __cplusplus
extern "C" {
#endif
    int LKH_main(int argc, char* argv[]);
#ifdef __cplusplus
}
#endif

// 实现AMNSSolver所有方法
/*
针对Ring Star Problem（RSP）​​，
RSP问题是在图中寻找一个环形路径，使得每个非节点都被分配到环上的某个节点，是的环边的花费和分配边的花费之和最小化。
根据问题定义，一个解包含两部分：

一个环（cycle），包含根节点v1和至少两个其他节点。

一个分配方案，将不在环上的节点分配到环上的某个节点（即分配给最近的环上节点）。

目标函数由两部分组成：环的成本（环上边的成本之和）和分配成本（每个不在环上的节点到其分配到的环上节点的成本之和）。

在局部搜索中，我们需要考虑如何表示一个解，以及如何通过改变环和分配来生成邻居解。

由于分配是由环决定的（每个非环节点被分配到环上最近的节点），所以一旦环确定了，分配也就确定了。因此，解可以由环上的节点集合（包括v1）以及环的顺序来表示。

但是，注意：分配成本是基于每个非环节点到环上节点的最小成本，所以如果环改变了，分配可能会改变。

因此，我们可以将解表示为环（一个节点序列，包括v1，且至少三个节点，包括v1和至少两个其他节点）。

N1: 2-opt（环内邻域，细粒度）
N2: 3-opt（环内邻域，交换两个节点的位置）
N3: 插入一个节点（环上节点变化）
N4: 删除一个节点（环上节点变化）
N5: 交换节点（环上节点变化，用一个非环节点替换环上一个节点）
*/
/*
a: 3 5 7 9
*/
AMNSSolver::AMNSSolver(const RSPGraph& g, int a, string filename)
    : graph(g), alpha(a), filename(filename) {

}


void AMNSSolver::solve(int benchmark_opt, int MAX_ITER) {
    
    double min_cost = INF;
    glo_benchmark_opt = benchmark_opt;
    build_static_cache_cloest();
    // 初始化
    Solution s = greedy_construction_all_nodes();
    best = s;
    min_cost = best.total_cost();
	set_SA_parameters_based_on_problem_size(); // 根据问题规模设置SA参数
        Solution neighbor = variable_neighborhood_VNS3(s, benchmark_opt); //variable_neighborhood_VNS3_SA  variable_neighborhood_VNS3
        evaluate(neighbor);

        if (!isSolutionValid(neighbor, graph)) {
            cout << "解无效" << endl;
			return;
        }
        best = neighbor;
        double neighbor_cost = best.total_cost();

            // 检查是否达到基准最优
            if (neighbor_cost == benchmark_opt) {
                best = neighbor;
                cout << "★★★★ 达到基准最优 " << benchmark_opt  << endl;
				return;
            }
}


void AMNSSolver::build_static_cache_cloest() {
    for (int u = 0; u < graph.nodes.size(); ++u) {
        vector<pair<double, int>> all_distances;

        // 计算到所有其他节点的距离（排除自身）
        for (int v = 0; v < graph.nodes.size(); ++v) {
            if (u != v) {
                all_distances.emplace_back(graph.routing_cost[u][v], v);
            }
        }

        // 按距离升序排序（默认按pair的first从小到大排序）
        sort(all_distances.begin(), all_distances.end());

        // 缓存所有节点（从最近到最远）
        static_near_cache_closet[u].clear();
        for (const auto& p : all_distances) {
            static_near_cache_closet[u].push_back(p.second);
        }
    }
}
Solution AMNSSolver::getBestSolution() {
    return best; // 返回当前最优解 
}

// 贪心策略将所有节点加入环中的初始解生成
Solution AMNSSolver::greedy_construction_all_nodes() {
    Solution s;
    s.initialize(graph.nodes.size());

    // 1. 初始环只包含根节点
    //s.ring.push_back(graph.depot);
    s.in_ring[graph.depot] = true;

    cout << "贪心构造初始解，总节点数: " << graph.nodes.size() << endl;

    // 2. 创建候选节点列表（排除根节点）
    vector<int> candidates;
    for (int i = 0; i < graph.nodes.size(); ++i) {
        if (i != graph.depot) {
            candidates.push_back(i);
        }
    }
    shuffle(candidates.begin(), candidates.end(), rng);
    cout << "候选节点数: " << candidates.size() << endl;

    // 3. 贪心插入所有节点
    while (!candidates.empty()) {
        double best_delta = INF;
        int best_node = -1;
        int best_position = -1;

        // 遍历所有候选节点，找到最佳插入位置
        for (int v : candidates) {
            // 对于每个候选节点，找到环中的最佳插入位置
            for (int i = 0; i < s.ring.size(); ++i) {
                int next = (i + 1) % s.ring.size();

                // 计算插入成本增量
                double delta = graph.routing_cost[s.ring[i]][v]
                    + graph.routing_cost[v][s.ring[next]]
                        - graph.routing_cost[s.ring[i]][s.ring[next]];

                    if (delta < best_delta) {
                        best_delta = delta;
                        best_node = v;
                        best_position = i + 1; // 插入在i之后
                    }
            }
        }

        // 插入最佳节点
        if (best_node != -1) {
            // 执行插入
            s.ring.insert(s.ring.begin() + best_position, best_node);
            s.in_ring[best_node] = true;

            // 从候选列表中移除
            candidates.erase(remove(candidates.begin(), candidates.end(), best_node), candidates.end());

            // 增量更新路由成本
            s.routing_cost += alpha * best_delta;

            if (s.ring.size() % 100 == 0) {
                cout << "已插入 " << s.ring.size() << "/" << graph.nodes.size()
                    << " 个节点，当前路由成本: " << s.routing_cost << endl;
            }
        }
        else {
            break; // 没有找到合适的节点，退出循环
        }
    }

    // 4. 所有节点都在环上，分配成本为0（节点自分配）
    s.assign_cost = 0;
    for (int i = 0; i < graph.nodes.size(); ++i) {
        if (s.in_ring[i]) {
            s.assignments[i] = i; // 环上节点自分配
        }
    }

    cout << "贪心构造完成，环大小: " << s.ring.size()
        << ", 路由成本: " << s.routing_cost
        << ", 总成本: " << s.total_cost() << endl;

    return s;
}




// 动态顺序VND实现
Solution AMNSSolver::local_search_FULL_VND(Solution s) {
    // 初始化统计
    
    vector<int> operator_order = { 6,5, 0, 1, 3, 4 };;//{   6 , 0, 1, 3, 4 }
    Solution current_best = s;
    int k = 0;
    int total_improves = 0;

    cout << endl;
    cout << "VND开始, 初始成本: " << current_best.total_cost() << endl;
        for (k = 0; k < params.local_search_size; ) {
            int op_index = operator_order[k];
            Solution neighbor = apply_local_search_operator(current_best, op_index);//apply_tabu_local_search_operator   apply_local_search_operator
            if (neighbor.total_cost() < current_best.total_cost()) {
                //k = 0;
                total_improves++;
                current_best = neighbor;
            }
            
            else {
				k++; // 尝试下一个邻域
            }
        }
        cout << "VND完成, 最终成本: " << current_best.total_cost()
            << ", 总改进次数: " << total_improves << endl;
    return current_best;
}

//退火
Solution AMNSSolver::variable_neighborhood_VNS3_SA(Solution s, int benchmark_opt) {
    Solution current = s;
    Solution best_sol = current;
    current_best_SA = current; // 初始化SA当前最优

    // 初始化模拟退火
    if (params.enable_simulated_annealing) {
        initialize_simulated_annealing();
    }

    // 时间控制
    auto start_time = chrono::steady_clock::now();
    int total_iterations = 0;
    int max_iterVNS = params.max_iterVNS;

    vector<int> shaking_order = { 0, 1, 2, 3 };
    int no_improve_count = 0;
    const int MAX_NO_IMPROVE = 60;

    while (true) {
        // 时间检查
        if (chrono::duration_cast<chrono::seconds>(
            chrono::steady_clock::now() - start_time).count() > params.max_time) {
            cout << "时间到，退出搜索" << endl;
            return best_sol;
        }

        int k = 1;
        while (true) {
            total_iterations++;

            // 随机选择shaking算子
            shuffle(shaking_order.begin(), shaking_order.end(), rng);

            for (int operator_index : shaking_order) {
                // 时间检查
                if (chrono::duration_cast<chrono::seconds>(
                    chrono::steady_clock::now() - start_time).count() > params.max_time) {
                    return best_sol;
                }

                cout << k << "时间: "
                    << chrono::duration_cast<chrono::seconds>(
                        chrono::steady_clock::now() - start_time).count() << endl << "=== 迭代 " << total_iterations
                    << ", k=" << ", 温度=" << current_temperature << " ===" << endl;

                // 扰动阶段
                Solution shaken_solution = apply_shaking_operator(current, operator_index, k);
                cout << "Shaking后: ring-size=" << shaken_solution.ring.size()
                    << ", cost=" << shaken_solution.total_cost();

                // 局部搜索阶段
                Solution local_optima = local_search_FULL_VND(shaken_solution);
                double new_cost = local_optima.total_cost();
                double current_cost = current.total_cost();
                double best_cost = best_sol.total_cost();

                cout << ", VND后: cost=" << new_cost
                    << ", 当前最优: " << best_cost << endl;

                // 模拟退火接受准则
                bool accept_solution = false;
                string accept_reason = "";

                if (params.enable_simulated_annealing) {
                    // 使用SA接受准则
                    accept_solution = accept_worse_solution(current_cost, new_cost, current_temperature);

                    if (new_cost < current_cost) {
                        accept_reason = "改进解";
                    }
                    else if (accept_solution) {
                        accept_reason = "SA概率接受恶化解";
                    }
                    else {
                        accept_reason = "SA拒绝恶化解";
                    }
                }
                else {
                    // 原始VNS接受准则（只接受改进解）
                    accept_solution = (new_cost < current_cost);
                    accept_reason = accept_solution ? "改进解" : "拒绝恶化解";
                }

                // 更新当前解
                if (accept_solution) {
                    Solution previous_current = current;
                    current = local_optima;

                    // 更新全局最优
                    if (current.total_cost() < best_sol.total_cost()) {
                        best_sol = current;
                        no_improve_count = 0;
                        cout << "★ 新全局最优: " << best_sol.total_cost()
                            << " ★ (" << accept_reason << ")" << endl;

                        // 使用LKH优化新找到的全局最优解
                        optimize_ring_with_lkh(best_sol);
                    }
                    else {
                        no_improve_count++;
                        cout << "接受新当前解: " << current.total_cost()
                            << " (" << accept_reason << ")" << endl;
                    }

                    // 重置k值（VNS标准策略）
                    k = 1;

                    // 检查是否达到基准最优
                    if (best_sol.total_cost() == benchmark_opt) {
                        cout << "★★★★ 达到基准最优 " << benchmark_opt << " ★★★★" << endl;
                        return best_sol;
                    }
                }
                else {
                    no_improve_count++;
                    cout << "拒绝新解 (" << accept_reason << ")" << endl;
                    k++; // 尝试下一个邻域
                }

                // 更新模拟退火温度
                if (params.enable_simulated_annealing) {
                    update_temperature();
                }

                
            }
        }

        // 温度过低时重新初始化SA（避免过早收敛）
        if (params.enable_simulated_annealing && current_temperature <= params.final_temperature * 2) {
            cout << "温度过低，重新初始化SA..." << endl;
            initialize_simulated_annealing();

            // 同时进行解的重初始化
            current = diversify_solution(best_sol);
        }
    }

    cout << "总迭代次数: " << total_iterations
        << ", 最终成本: " << best_sol.total_cost() << endl;

    return best_sol;
}

Solution AMNSSolver::variable_neighborhood_VNS3(Solution s, int benchmark_opt) {
    Solution current = s;
    // 论文参数：时间限制和扰动强度
    double max_time = params.max_time;  // 60秒时间限制（论文标准）
    int k_max = params.k_max;           // 最大扰动强度（根据论文实验确定）

    Solution newS = greedy_construction_all_nodes();//  greedy_construction_all_nodes() 
    current = newS;
    optimize_ring_with_lkh(current);
    Solution best_sol = current;
    // 时间控制
    auto start_time = chrono::steady_clock::now();
    int total_iterations = 0;
    int max_iterVNS = params.max_iterVNS;

    vector<int> shaking_order = { 0, 1, 2,3 };
    while (true) {
        // 初始化
        int k = 1;
        current_iteration = 0;
		cout << endl << "================= 新一轮VNS开始 ===================" << endl;
        while (true) {
            total_iterations++;
            shuffle(shaking_order.begin(), shaking_order.end(), rng);
            for (int i = 0; i < shaking_order.size(); i++) {
                if (chrono::duration_cast<chrono::seconds>(
                    chrono::steady_clock::now() - start_time).count() > max_time)
                {
                    return best_sol;
                }
                current_iteration++; // 更新迭代计数
                int operator_index = shaking_order[i];
                cout << endl;
                Solution shaken_solution = apply_shaking_operator(current, operator_index, k);
                cout << "shaking后：ring-size: " << shaken_solution.ring.size() << ", no-ring-size: " << graph.nodes.size() - shaken_solution.ring.size();
                Solution local_optima = local_search_FULL_VND(shaken_solution);

                double delta_cost = local_optima.total_cost() - current.total_cost();


                cout << "时间: "
                    << chrono::duration_cast<chrono::seconds>(
                        chrono::steady_clock::now() - start_time).count()
                    << "s,k=" << k << ", shaking算子" << operator_index
                    << ", 当前解: " << current.total_cost()
                    << ", 新解局部最优: " << local_optima.total_cost()
                    << ", 全局最优: " << best_sol.total_cost();

                // 改进接受准则
                if (local_optima.total_cost() <current.total_cost()+ current.total_cost()*0.0003) {
                    current = local_optima;
                    optimize_ring_with_lkh(current); // LKH优化
                    // 更新全局最优
                    if (current.total_cost() < best_sol.total_cost()) {
                        k = 1;
                        best_sol = current;
                        cout << "★ring-size: " << best_sol.ring.size() << ", no-ring-size: " << graph.nodes.size() - best_sol.ring.size();
                        cout << " ★新全局最优" << endl;
                    }
                    else {
                        k++;
                        //optimize_ring_with_lkh(current); // LKH优化
                        cout << " ★改进当前解" << endl;
                    }
                    // 提前终止检查
                    if (best_sol.total_cost() == benchmark_opt) {
                        cout << "★ 达到基准最优 " << benchmark_opt << "，提前终止 ★" << endl;
                        cout << "总迭代次数: " << total_iterations
                            << ", 最终成本: " << best_sol.total_cost() << endl;
                        return best_sol;
                    }
                }
                else {
                    cout << "ring-size: " << best_sol.ring.size() << ", no-ring-size: " << graph.nodes.size() - best_sol.ring.size();
                    cout << " →未改进" << endl;
                    k++;
                }

            }
        }
    }
    cout << "总迭代次数: " << total_iterations
        << ", 最终成本: " << best_sol.total_cost() << endl;

    return best_sol;
}


Solution AMNSSolver::apply_shaking_operator(Solution s, int operator_index, int k) {
    Solution s2=s;
    switch (operator_index) {
    case 0:
        s2 = random_node_swap(s, k);//random_node_swap
        break;
    case 1:
        s2 = random_node_swap(s, k);//multiple_node_operation
        break;
    case 2:
        s2 = assignment_perturbation(s, k);//assignment_perturbation
        break;
    case 3:
        s2 = random_node_swap(s, k);//remove_most_assigned_nodes
        break;
    default:
        s2 = random_node_swap(s, k);
        break;
    }
    evaluate(s2);
    return s2;
}

Solution AMNSSolver::apply_local_search_operator(Solution s, int operator_index) {
    Solution result=s;
    if (!validate_solution2(result)) {
        cout << "无效初始解"  << endl;
    }
    int size = 0;
    if (alpha == 5) {
		size = 35;
    }
    else if (alpha == 7) {
		size = 30;
    }
    else {
        size = 25;
    }
    switch (operator_index) {// { 0,  1,6, 3, 4 };
    case 0: result = exhaustive_two_opt(s); break;
	case 1: result = randomized_three_opt(s); break;
    case 3: result = exhaustive_add_node(s); break;
    case 4: result = exhaustive_drop_node(s); break; //s.ring.size()*params.batch_intelligent_removal_rito * alpha / (10 - alpha)
    case 5: result = cluster_insertion_operator(s); break;
    case 6: result = batch_intelligent_removal(s, size); break;    // 新增批量删除
    default: result = exhaustive_two_opt(s);
    }

    return result;
}


//all

//shaking


Solution AMNSSolver::random_node_swap(Solution s, int k) {
    Solution perturbed = s;
    const int min_ring_size = 3;

    // 扰动强度决定操作次数：k=1时1次，k=2时2次，以此类推
    int operations = (int)perturbed.ring.size() * 0.4;

    for (int op = 0; op < operations; ++op) {
        vector<int> ring_nodes, non_ring_nodes;

        // 收集环上节点（排除depot）和非环节点
        for (int i = 0; i < graph.nodes.size(); ++i) {
            if (perturbed.in_ring[i] && i != graph.depot) {
                ring_nodes.push_back(i);
            }
            else if (!perturbed.in_ring[i]) {
                non_ring_nodes.push_back(i);
            }
        }

        if (ring_nodes.empty() || non_ring_nodes.empty()) break;

        // 随机选择操作类型
        int operation_type = rand() % 3;

        switch (operation_type) {
        case 0: // 删除环上节点
            if (perturbed.ring.size() > min_ring_size) {
                int node_to_remove = ring_nodes[rand() % ring_nodes.size()];
                remove_node(perturbed, node_to_remove);
            }
            break;

        case 1: // 添加非环节点
            if (!non_ring_nodes.empty()) {
                int node_to_add = non_ring_nodes[rand() % non_ring_nodes.size()];
                insert_node(perturbed, node_to_add);
            }
            break;

        case 2: // 替换操作：删除一个环上节点，添加一个非环节点
            if (perturbed.ring.size() > min_ring_size && !non_ring_nodes.empty()) {
                int node_to_remove = ring_nodes[rand() % ring_nodes.size()];
                int add_node = non_ring_nodes[rand() % non_ring_nodes.size()];
                remove_node(perturbed, node_to_remove);
                insert_node(perturbed, add_node);
            }
            break;
        }
    }

    return perturbed;
}


Solution AMNSSolver::multiple_node_operation(Solution s, int k) {
    Solution perturbed = s;
    const int min_ring_size = 3;
	int ratio = 0.3; // 扰动比例
    if (alpha == 5) {
		ratio = 0.3;
	}
	else if (alpha == 7) {
		ratio = 0.3;
	}
    else {
        ratio = 0.3;
    }
    // 根据k值决定操作的节点数量
    //int node_count = min(1 + k * 3, (int)perturbed.ring.size() );//
	int node_count = (int)perturbed.ring.size() * ratio; // 扰动10%的环节点数
    vector<int> ring_nodes, non_ring_nodes;
    for (int i = 0; i < graph.nodes.size(); ++i) {
        if (perturbed.in_ring[i] && i != graph.depot) {
            ring_nodes.push_back(i);
        }
        else if (!perturbed.in_ring[i]) {
            non_ring_nodes.push_back(i);
        }
    }

    // 批量删除节点
    int delete_count = min(node_count , (int)ring_nodes.size());
    for (int i = 0; i < delete_count && perturbed.ring.size() > min_ring_size; ++i) {
        int node_to_remove = ring_nodes[rand() % ring_nodes.size()];
        remove_node(perturbed, node_to_remove);
        // 更新ring_nodes（移除已删除的节点）
        ring_nodes.erase(remove(ring_nodes.begin(), ring_nodes.end(), node_to_remove),
            ring_nodes.end());
    }

    // 批量添加节点
    int add_count = min(node_count , (int)non_ring_nodes.size());
    for (int i = 0; i < add_count; ++i) {
        int node_to_add = non_ring_nodes[rand() % non_ring_nodes.size()];
        insert_node(perturbed, node_to_add);
        // 更新non_ring_nodes
        non_ring_nodes.erase(remove(non_ring_nodes.begin(), non_ring_nodes.end(), node_to_add),
            non_ring_nodes.end());
    }

    return perturbed;
}


Solution AMNSSolver::assignment_perturbation(Solution s, int k) {
    Solution perturbed = s;

    // 随机选择一部分非环节点，重新分配它们到不同的环上节点
    int reassign_count = min(20 + k * 5, (int)graph.nodes.size() / 2);

    vector<int> non_ring_nodes;
    for (int i = 0; i < graph.nodes.size(); ++i) {
        if (!perturbed.in_ring[i]) {
            non_ring_nodes.push_back(i);
        }
    }

    shuffle(non_ring_nodes.begin(), non_ring_nodes.end(), rng);

    for (int i = 0; i < (int)non_ring_nodes.size(); ++i) {
        int u = non_ring_nodes[i];

        // 随机选择一个环上节点（可能不是最优的）
        vector<int> ring_candidates;
        for (int v : perturbed.ring) {
            if (v != graph.depot) {
                ring_candidates.push_back(v);
            }
        }

        if (!ring_candidates.empty()) {
            int new_assign = ring_candidates[rand() % ring_candidates.size()];

            // 更新分配关系
            auto& old_list = perturbed.assignment_map[perturbed.assignments[u]];
            old_list.erase(remove(old_list.begin(), old_list.end(), u), old_list.end());

            perturbed.assignments[u] = new_assign;
            perturbed.assignment_map[new_assign].push_back(u);
        }
    }

    // 重新计算分配成本
    double new_assign_cost = 0;
    for (int u = 0; u < graph.nodes.size(); ++u) {
        if (!perturbed.in_ring[u]) {
            new_assign_cost += (10 - alpha) * graph.assign_cost[u][perturbed.assignments[u]];
        }
    }
    perturbed.assign_cost = new_assign_cost;

    return perturbed;
}

Solution AMNSSolver::remove_most_assigned_nodes(Solution s, int k) {
    Solution perturbed = s;
    const int min_ring_size = 3; // 确保环至少包含3个节点

    // 如果环大小已经是最小值，直接返回
    if (perturbed.ring.size() <= min_ring_size) {
        cout << "环大小已达最小值，跳过删除操作" << endl;
        return perturbed;
    }

    // 步骤1: 统计每个环节点的被分配数量
    vector<pair<int, int>> assignment_counts; // (节点ID, 被分配数量)

    for (int ring_node : perturbed.ring) {
        if (ring_node == graph.depot) {
            continue; // 跳过根节点，通常不能删除
        }

        int count = 0;
        if (perturbed.assignment_map.count(ring_node)) {
            count = perturbed.assignment_map[ring_node].size();
        }
        assignment_counts.emplace_back(ring_node, count);
    }

    // 如果没有可删除的节点，直接返回
    if (assignment_counts.empty()) {
        cout << "没有可删除的环节点" << endl;
        return perturbed;
    }

    // 步骤2: 按被分配数量降序排序
    sort(assignment_counts.begin(), assignment_counts.end(),
        [](const pair<int, int>& a, const pair<int, int>& b) {
            return a.second > b.second; // 降序排列
        });
    int node_count = (int)perturbed.ring.size() * 0.2; // 扰动10%的环节点数
    // 步骤3: 确定实际要删除的节点数量
    int actual_k = min(k, static_cast<int>(assignment_counts.size()));
    actual_k = min(actual_k, static_cast<int>(perturbed.ring.size()) - min_ring_size);

    if (actual_k <= 0) {
        cout << "没有足够的节点可供删除" << endl;
        return perturbed;
    }

    cout << "准备删除前 " << actual_k << " 个被分配最多的环节点" << endl;

    // 步骤4: 批量删除节点
    vector<int> nodes_to_remove;
    for (int i = 0; i < node_count; ++i) {
        nodes_to_remove.push_back(assignment_counts[i].first);
    }

    // 步骤5: 执行删除操作（需要特殊处理以确保解的有效性）
    for (int node_to_remove : nodes_to_remove) {
        if (perturbed.ring.size() <= min_ring_size) {
            break; // 确保环大小不低于最小值
        }

        // 检查节点是否仍在环中
        if (perturbed.in_ring[node_to_remove]) {
            remove_node(perturbed, node_to_remove);
        }
    }

    // 步骤6: 验证解的有效性
    if (!validate_solution2(perturbed)) {
        cout << "警告: 删除操作后解无效，尝试修复" << endl;
        // 如果解无效，回退到原始解
        perturbed = s;
    }

    cout << "删除操作完成，新环大小: " << perturbed.ring.size()
        << ", 被删除节点数: " << nodes_to_remove.size() << endl;

    return perturbed;
}

void AMNSSolver::remove_node(Solution& s, int v) {
    // 找到节点在环中的位置
    auto it = find(s.ring.begin(), s.ring.end(), v);
    if (it == s.ring.end()) return;

    int pos = distance(s.ring.begin(), it);
    int prev_node = s.ring[(pos - 1 + s.ring.size()) % s.ring.size()];
    int next_node = s.ring[(pos + 1) % s.ring.size()];

    // 计算路由成本变化
    double routing_delta = alpha * (graph.routing_cost[prev_node][next_node]
        - graph.routing_cost[prev_node][v]
            - graph.routing_cost[v][next_node]);
    s.routing_cost += routing_delta;

    // 删除节点
    s.ring.erase(it);
    s.in_ring[v] = false;

    // 重新分配受影响的节点
    reallocate_affected_nodes(s, v);
}

void AMNSSolver::reallocate_affected_nodes(Solution& s, int removed_node) {


    // 包括被删除节点本身和原来分配到这个节点的所有非环节点
    // 收集受影响节点
    vector<int> affected_nodes;
    if (s.assignment_map.count(removed_node)) {
        affected_nodes = move(s.assignment_map[removed_node]);
    }
    s.assignment_map.erase(removed_node);

    affected_nodes.push_back(removed_node);

    // 批量重分配
    for (int u : affected_nodes) {
        if (s.in_ring[u]) continue;  // 环上节点不需要重分配

        int new_assign = find_closest_ring_node(s, u);

        // 更新分配关系
        if (s.assignments[u] != new_assign) {
            // 从原分配列表中移除
            if (s.assignment_map.count(s.assignments[u])) {
                auto& old_list = s.assignment_map[s.assignments[u]];
                old_list.erase(remove(old_list.begin(), old_list.end(), u), old_list.end());
            }

            // 添加到新分配列表
            s.assignments[u] = new_assign;
            s.assignment_map[new_assign].push_back(u);
        }
    }

    // 重新计算分配成本
    double new_assign_cost = 0;
    for (int u = 0; u < graph.nodes.size(); ++u) {
        if (!s.in_ring[u]) {
            new_assign_cost += (10 - alpha) * graph.assign_cost[u][s.assignments[u]];
        }
    }
    s.assign_cost = new_assign_cost;
}

void AMNSSolver::improved_remove_node(Solution& s, int node_to_remove) {
    // 1. 基础检查：确认要删除的节点在环上且不是根节点
    auto it = find(s.ring.begin(), s.ring.end(), node_to_remove);
    if (it == s.ring.end() || node_to_remove == graph.depot) return;

    // 2. 记录删除节点的位置和其原分配节点列表
    int pos = distance(s.ring.begin(), it);
    int prev_node = s.ring[(pos - 1 + s.ring.size()) % s.ring.size()];
    int next_node = s.ring[(pos + 1) % s.ring.size()];

    vector<int> affected_nodes;
    if (s.assignment_map.count(node_to_remove)) {
        affected_nodes = s.assignment_map[node_to_remove]; // 获取原分配列表的副本
    }
    affected_nodes.push_back(node_to_remove); // 包括被删除节点本身
    s.assignment_map.erase(node_to_remove);

    // 3. 计算并应用删除节点本身带来的路由成本变化
    double routing_delta_remove = alpha * (graph.routing_cost[prev_node][next_node]
        - graph.routing_cost[prev_node][node_to_remove]
            - graph.routing_cost[node_to_remove][next_node]);
    s.routing_cost += routing_delta_remove;

    // 4. 从环中移除该节点
    s.ring.erase(it);
    s.in_ring[node_to_remove] = false;

    // 5. 对每个受影响节点，选择最优方案（分配或插入）
    double total_assign_cost_delta = 0.0;

    for (int u : affected_nodes) {
        if (s.in_ring[u]) continue; // 环上节点（除了被删的）不需要处理

        // 方案A: 分配给最佳的现有环节点
        int best_assign_node = find_closest_ring_node(s, u);
        double cost_assign = (10 - alpha) * graph.assign_cost[u][best_assign_node];

        // 方案B: 插入到被删除节点的位置
        // 计算插入到 prev_node 和 next_node 之间的成本
        double cost_insert = alpha * (graph.routing_cost[prev_node][u] + graph.routing_cost[u][next_node] - graph.routing_cost[prev_node][next_node]);
        // 注意：如果u被插入环，它就不再产生分配成本
        cost_insert -= (10 - alpha) * graph.assign_cost[u][best_assign_node]; // 减去它即将的分配成本

        // 决策：选择总成本更低的方案
        if (cost_insert < cost_assign) {
            // 执行插入方案
            s.ring.insert(s.ring.begin() + pos, u); // 在原位置插入
            s.in_ring[u] = true;
            s.routing_cost += cost_insert;
            // 该节点成为环上节点，自分配
            s.assignments[u] = u;
            // 注意：不需要将其加入 assignment_map，因为环上节点不在此映射中
        }
        else {
            // 执行分配方案
            s.assignments[u] = best_assign_node;
            s.assignment_map[best_assign_node].push_back(u);
            total_assign_cost_delta += (10 - alpha) * (graph.assign_cost[u][best_assign_node] - graph.assign_cost[u][s.assignments[u]]);
        }
    }

    // 6. 更新总分配成本
    s.assign_cost += total_assign_cost_delta;
}

// 在AMNSSolver类中添加以下函数
Solution AMNSSolver::cluster_insertion_operator(Solution s) {
    Solution best_solution = s;
    double best_cost = s.total_cost();
    int depot = graph.depot;
    const int MAX_INSERTION_SIZE = 10; // 防止插入节点过多导致组合爆炸

    // 1. 遍历环中每个非Depot节点作为候选v1
    for (int idx = 0; idx < s.ring.size(); ++idx) {
        int v1 = s.ring[idx];
        if (v1 == depot) continue;

        // 检查v1是否有分配节点且数量可控
        if (s.assignment_map.find(v1) == s.assignment_map.end() ||
            s.assignment_map.at(v1).empty() ||
            s.assignment_map.at(v1).size() > MAX_INSERTION_SIZE) {
            continue;
        }

        Solution temp = s;
        vector<int> L = temp.assignment_map[v1]; // 获取v1的分配节点列表

        // 2. 记录v1的环位置及邻接节点
        auto it = find(temp.ring.begin(), temp.ring.end(), v1);
        int pos = distance(temp.ring.begin(), it);
        int prev_node = temp.ring[(pos - 1 + temp.ring.size()) % temp.ring.size()];
        int next_node = temp.ring[(pos + 1) % temp.ring.size()];

        // 3. 移除v1并更新路由成本
        temp.ring.erase(temp.ring.begin() + pos);
        temp.in_ring[v1] = false;
        double routing_delta_remove = alpha * (
            graph.routing_cost[prev_node][next_node] -
            graph.routing_cost[prev_node][v1] -
            graph.routing_cost[v1][next_node]
            );
        temp.routing_cost += routing_delta_remove;

        //// 4. 重新分配v1自身
        //int new_assign_v1 = find_closest_ring_node(temp, v1);
        //double assign_delta_v1 = (10 - alpha) * graph.assign_cost[v1][new_assign_v1]; // v1原自分配成本为0
        //temp.assignments[v1] = new_assign_v1;
        //temp.assignment_map[new_assign_v1].push_back(v1);
        //temp.assignment_map.erase(v1); // 移除v1的分配列表
        //temp.assign_cost += assign_delta_v1;

        temp.assignment_map.erase(v1); // 移除v1的分配列表

        // 5. 找到节点集L的最佳插入顺序
        vector<int> best_order = L;
        double min_insert_cost = INF;
        
            // 大规模时使用启发式：按与v1的距离排序
            sort(L.begin(), L.end(), [&](int a, int b) {
                return graph.routing_cost[prev_node][a] < graph.routing_cost[prev_node][b];
                });
            best_order = L;
            min_insert_cost = calculate_insertion_cost(prev_node, next_node, best_order);
        

        // 6. 插入节点并更新路由成本
        temp.ring.insert(temp.ring.begin() + pos, best_order.begin(), best_order.end());
        for (int u : best_order) temp.in_ring[u] = true;
        temp.routing_cost += alpha * min_insert_cost;

        // 7. 更新L中节点的分配（变为自分配）
        double assign_delta_L = 0;
        for (int u : best_order) {
            assign_delta_L -= (10 - alpha) * graph.assign_cost[u][v1]; // 原分配成本消失
            temp.assignments[u] = u; // 自分配
        }
        temp.assign_cost += assign_delta_L;

        // 8. 重新分配所有非环节点以适应环变化
        reallocateNoCircle(temp);
        // 验证解并更新最优
        if (validate_solution2(temp) && temp.total_cost() < best_cost) {
            best_solution = temp;
            best_cost = temp.total_cost();
			return best_solution; // 发现改进立即返回
        }
    }
    return best_solution;
}

// 辅助函数：计算插入序列的成本
double AMNSSolver::calculate_insertion_cost(int prev, int next, const vector<int>& seq) {
    if (seq.empty()) return 0;
    double cost = graph.routing_cost[prev][seq[0]] + graph.routing_cost[seq.back()][next];
    for (size_t i = 0; i < seq.size() - 1; i++) {
        cost += graph.routing_cost[seq[i]][seq[i + 1]];
    }
    return cost - graph.routing_cost[prev][next]; // 减去被替换的边
}

Solution AMNSSolver::exhaustive_add_node(Solution s) {
    Solution best_solution = s;
    double min_cost = s.total_cost();//不一定比s小，后面以概率接受 如果比s小 换成s.total_cost()
    int node = 0;
    // 遍历所有非环节点
    for (int v = 0; v < graph.nodes.size(); ++v) {

        if (!s.in_ring[v] && v != graph.depot) {
            Solution temp = s;
            insert_node(temp, v);//TODO 可优化增量

            evaluate(temp);
            if (temp.total_cost() < min_cost) {
                node = v;
                min_cost = temp.total_cost();
                best_solution = temp;
            }   
        }
    }
    return best_solution;
}



Solution AMNSSolver::exhaustive_drop_node(Solution s) {
    if (s.ring.size() <= 3) return s;

    Solution best_solution = s;
    double min_cost = s.total_cost();
    for (size_t pos = 1; pos < s.ring.size(); ++pos) {
        int v = s.ring[pos];
        if (v == graph.depot) continue;

        Solution temp = s;
		remove_node(temp, v);//TODO 可优化增量
		evaluate(temp);
        if (temp.total_cost() < min_cost) {
            min_cost = temp.total_cost();
            best_solution = temp;
        }
    }
    return best_solution;
}


Solution AMNSSolver::exhaustive_two_opt(Solution s) {
    bool improved;
    int i = 1;
    do {
        improved = false;
        double best_delta = 0;
        int best_i = -1, best_j = -1;

        // 遍历所有可能的2-opt交换
        for (int i = 0; i < s.ring.size() - 1; ++i) {
            for (int j = i + 2; j < s.ring.size(); ++j) {
                // 计算交换前后的成本变化
                double delta = graph.routing_cost[s.ring[i]][s.ring[j]]
                    + graph.routing_cost[s.ring[i + 1]][s.ring[(j + 1) % s.ring.size()]]
                    - graph.routing_cost[s.ring[i]][s.ring[i + 1]]
                    - graph.routing_cost[s.ring[j]][s.ring[(j + 1) % s.ring.size()]];

                if (delta < best_delta) {
                    best_delta = delta;
                    best_i = i;
                    best_j = j;
                }
            }
        }

        // 执行最佳交换
        if (best_delta < -1e-6) { // 考虑浮点误差
            reverse(s.ring.begin() + best_i + 1, s.ring.begin() + best_j + 1);
            s.routing_cost += alpha * best_delta;
            improved = true;
        }
    } while (improved);

    return s;
}


Solution AMNSSolver::randomized_three_opt(Solution s) {
    const int MAX_ATTEMPTS = 50;
    int attempts = 0;
    bool improved;
    const int n = s.ring.size();
    if (n < 6) return s;

    do {
        improved = false;
        int i = rand() % (n - 5);
        int j = (i + 2 + rand() % (n - i - 4)) % n;
        int k = (j + 2 + rand() % (n - j - 2)) % n;

        if (i > j) swap(i, j);
        if (j > k) swap(j, k);
        if (i > j) swap(i, j);

        int a = s.ring[i];
        int b = s.ring[(i + 1) % n];
        int c = s.ring[j];
        int d = s.ring[(j + 1) % n];
        int e = s.ring[k];
        int f = s.ring[(k + 1) % n];

        double original = alpha * (graph.routing_cost[a][b] +
            graph.routing_cost[c][d] +
            graph.routing_cost[e][f]);

        vector<double> costs(7, 0);
        vector<vector<int>> segments(7, s.ring);

        // 情况0: 原始情况
        costs[0] = original;

        // 情况1: 2-opt (i-j)
        reverse(segments[1].begin() + i + 1, segments[1].begin() + j + 1);
        costs[1] = alpha * (graph.routing_cost[a][c] +
            graph.routing_cost[b][e] +
            graph.routing_cost[d][f]);

        // 情况2: 2-opt (j-k)
        reverse(segments[2].begin() + j + 1, segments[2].begin() + k + 1);
        costs[2] = alpha * (graph.routing_cost[a][b] +
            graph.routing_cost[c][e] +
            graph.routing_cost[d][f]);

        // 情况3: 2-opt (i-k)
        reverse(segments[3].begin() + i + 1, segments[3].begin() + k + 1);
        costs[3] = alpha * (graph.routing_cost[a][e] +
            graph.routing_cost[d][b] +
            graph.routing_cost[c][f]);

        // 情况4: 3-opt (保持方向)
        rotate(segments[4].begin() + i + 1,
            segments[4].begin() + j + 1,
            segments[4].begin() + k + 1);
        costs[4] = alpha * (graph.routing_cost[a][d] +
            graph.routing_cost[e][c] +
            graph.routing_cost[b][f]);

        // 情况5: 3-opt (反转中间段)
        reverse(segments[5].begin() + i + 1, segments[5].begin() + j + 1);
        rotate(segments[5].begin() + i + 1,
            segments[5].begin() + j + 1,
            segments[5].begin() + k + 1);
        costs[5] = alpha * (graph.routing_cost[a][d] +
            graph.routing_cost[e][b] +
            graph.routing_cost[c][f]);

        // 情况6: 3-opt (反转最后段)
        reverse(segments[6].begin() + j + 1, segments[6].begin() + k + 1);
        rotate(segments[6].begin() + i + 1,
            segments[6].begin() + j + 1,
            segments[6].begin() + k + 1);
        costs[6] = alpha * (graph.routing_cost[a][e] +
            graph.routing_cost[d][c] +
            graph.routing_cost[b][f]);

        int best_case = 0;
        double best_gain = 0;
        for (int c = 1; c < 7; ++c) {
            double gain = original - costs[c];
            if (gain > best_gain) {
                best_gain = gain;
                best_case = c;
            }
        }

        if (best_gain > 1e-6) {
            s.ring = segments[best_case];
            s.routing_cost -= best_gain; // 增量更新路由成本
            improved = true;
            attempts = 0;
        }
        else {
            attempts++;
        }
    } while (improved || attempts < MAX_ATTEMPTS);
    evaluate(s);
    return s;
}


// 执行单个顶点交换操作
bool AMNSSolver::perform_vertex_exchange(Solution& s1, int vj, int vl) {
    Solution s = s1;
    // 验证输入
    if (!s.in_ring[vj] || s.in_ring[vl]) {
        return false;
    }

    if (vj == graph.depot || vl == graph.depot) {
        return false; // 不能交换depot
    }

    // 步骤1: 从环中移除vj
    remove_node(s, vj);
    // 步骤2: 将vl插入环中最佳位置
    insert_node(s, vl);
  
    // 验证解的有效性
    if (!validate_solution2(s)) {
		cout << "交换操作后解无效，放弃此次交换: 移除" << vj << " 插入" << vl << endl;
        // 恢复原始状态
        return false;
    }

    return true;
}


void AMNSSolver::insert_node(Solution& s, int v) {
    double min_cost = INF;
    int best_pos = -1;
    int prev_node = -1, next_node = -1;

    // 寻找插入后路由成本最小的位置
    for (int i = 0; i < s.ring.size(); ++i) {
        int j = (i + 1) % s.ring.size();
        double delta = graph.routing_cost[s.ring[i]][v]
            + graph.routing_cost[v][s.ring[j]]
                - graph.routing_cost[s.ring[i]][s.ring[j]];

            if (delta < min_cost) {
                min_cost = delta;
                best_pos = i + 1;
                prev_node = s.ring[i];
                next_node = s.ring[j];
            }
    }

    // 增量更新路由成本
    double routing_delta = alpha * min_cost;
    s.routing_cost += routing_delta;

    // 插入节点
    s.ring.insert(s.ring.begin() + best_pos, v);
    s.in_ring[v] = true;
    
    // 新增：清理节点v原有的分配关系
    int old_assign = s.assignments[v]; // 记录v原有的分配目标
    if (s.assignment_map.count(old_assign)) {
        // 从旧分配列表中移除v
        auto& old_list = s.assignment_map[old_assign];
        old_list.erase(std::remove(old_list.begin(), old_list.end(), v), old_list.end());
        // 可选：如果列表为空，删除该条目
        if (old_list.empty()) {
            s.assignment_map.erase(old_assign);
        }
    }
    // 增量更新分配成本
    double assign_delta = 0.0;
    double origin_assign = (10 - alpha) * graph.assign_cost[v][s.assignments[v]];//v从非环入环，分配成本减少自己的
    // 更新v的分配值为自身（环上节点自分配）
    s.assignments[v] = v;
    // 注意：不需要将v添加到s.assignment_map[v]，因为环上节点不需要被分配列表跟踪

    for (int u = 0; u < graph.nodes.size(); ++u) {
        if (!s.in_ring[u] && graph.assign_cost[u][v] < graph.assign_cost[u][s.assignments[u]]) {
            // 计算分配成本变化
            double old_cost = (10 - alpha) * graph.assign_cost[u][s.assignments[u]];
            double new_cost = (10 - alpha) * graph.assign_cost[u][v];
            assign_delta += (new_cost - old_cost);

            // 更新分配关系
            auto& old_list = s.assignment_map[s.assignments[u]];
            old_list.erase(remove(old_list.begin(), old_list.end(), u), old_list.end());

            s.assignments[u] = v;
            s.assignment_map[v].push_back(u);
        }
    }
    s.assign_cost += assign_delta - origin_assign;
}


int AMNSSolver::find_closest_ring_node(const Solution& s, int u) {
    // 直接遍历预排序的缓存列表
    for (int v : static_near_cache_closet.at(u)) {
        if (s.in_ring[v]) {
            // 路由成本最低的环节点即分配成本最优解（正相关保证）
            return v;
        }
    }
    return graph.depot; // 返回depot保底

}


void AMNSSolver::reallocateNoCircle(Solution& s) {
    double assign_delta = 0.0;

    for (int u = 0; u < graph.nodes.size(); ++u) {
        if (!s.in_ring[u]) {
            int new_assign = find_closest_ring_node(s, u);
            if (new_assign != s.assignments[u]) {
                // 计算分配成本变化
                double old_cost = (10 - alpha) * graph.assign_cost[u][s.assignments[u]];
                double new_cost = (10 - alpha) * graph.assign_cost[u][new_assign];
                assign_delta += (new_cost - old_cost);

                // 更新分配关系
                auto& old_list = s.assignment_map[s.assignments[u]];
                old_list.erase(remove(old_list.begin(), old_list.end(), u), old_list.end());

                s.assignments[u] = new_assign;
                s.assignment_map[new_assign].push_back(u);
            }
        }
    }

    s.assign_cost += assign_delta;
}

void AMNSSolver::evaluate(Solution& s) {
    // 计算路由成本（环路径总长度）
    s.routing_cost = 0;
    for (int i = 0; i < s.ring.size(); ++i) {
        int j = (i + 1) % s.ring.size();
        s.routing_cost += alpha * graph.routing_cost[s.ring[i]][s.ring[j]];
    }

    // 计算分配成本（所有非环节点到最近环节点的成本）
    s.assign_cost = 0;
    for (int u = 0; u < graph.nodes.size(); ++u) {
        if (!s.in_ring[u]) {              // 只考虑非环节点
            int v = s.assignments[u];     // 获取当前分配的环节点
            s.assign_cost += (10 - alpha) * graph.assign_cost[u][s.assignments[u]];
        }
    }
}

bool AMNSSolver::isSolutionValid(const Solution& s, const RSPGraph& graph) {
    if (s.ring.size() < 3) {
        std::cerr << "Error: Ring size is less than 3." << std::endl;
        return false;
    }
    // Check 1: Depot must be in the ring
    if (std::find(s.ring.begin(), s.ring.end(), graph.depot) == s.ring.end()) {
        std::cerr << "Error: Depot " << graph.depot << " is not in the ring." << std::endl;
        return false;
    }

    // Check 2: Ring vertices must be unique
    std::unordered_set<int> ring_set(s.ring.begin(), s.ring.end());
    if (ring_set.size() != s.ring.size()) {
        std::cerr << "Error: Ring contains duplicate vertices." << std::endl;
        return false;
    }

    // Check 3: Consistency between in_ring and ring list
    for (int v : s.ring) {
        if (v < 0 || v >= graph.nodes.size() || !s.in_ring[v]) {
            std::cerr << "Error: Vertex " << v << " is in ring list but not marked in_ring." << std::endl;
            return false;
        }
    }
    // Check 5: Assignments for non-ring vertices must point to ring vertices
    for (int v = 0; v < graph.nodes.size(); v++) {
        if (!s.in_ring[v]) {
            int assign_to = s.assignments.at(v);
            if (assign_to < 0 || assign_to >= graph.nodes.size() || !s.in_ring[assign_to]) {
                std::cerr << "Error: Non-ring vertex " << v << " is assigned to invalid vertex " << assign_to << " (not in ring)." << std::endl;
                return false;
            }
        }
    }
    return true;
}

void AMNSSolver::optimize_ring_with_lkh(Solution& s) {
    double original_cost = s.total_cost();
    if (s.ring.size() < 3) return;

    // 创建临时目录
    const std::string temp_dir = "lkh_temp";
    std::filesystem::create_directories(temp_dir);

    // 生成唯一的文件名
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
    std::string timestamp = oss.str();

    const std::string tsp_path = temp_dir + "/ring_" + timestamp + ".tsp";
    const std::string par_path = temp_dir + "/config_" + timestamp + ".par";
    const std::string tour_path = temp_dir + "/ring_" + timestamp + ".tour";

    // 1. 创建TSP问题文件 - 使用TSPLIB标准格式
    std::ofstream tsp_file(tsp_path);
    if (!tsp_file) {
        std::cerr << "无法创建TSP文件: " << tsp_path << std::endl;
        return;
    }

    // 写入TSPLIB标准头部
    tsp_file << "NAME: " << filename << "\n";
    tsp_file << "TYPE: TSP\n";
    tsp_file << "COMMENT: Optimized ring by LKH\n";
    tsp_file << "DIMENSION: " << s.ring.size() << "\n";
    tsp_file << "EDGE_WEIGHT_TYPE: EUC_2D\n"; // 使用欧几里得距离

    // 写入节点坐标部分
    tsp_file << "NODE_COORD_SECTION\n";
    for (int i = 0; i < s.ring.size(); ++i) {
        int node_id = s.ring[i];
        tsp_file << (i + 1) << " "
            << graph.nodes[node_id].first << " "
            << graph.nodes[node_id].second << "\n";
    }
    tsp_file << "EOF\n";
    tsp_file.close();

    // 2. 创建参数文件（保持不变）
    std::ofstream par_file(par_path);
    if (!par_file) {
        std::cerr << "无法创建参数文件: " << par_path << std::endl;
        return;
    }

    par_file << "PROBLEM_FILE = " << tsp_path << "\n";
    par_file << "TOUR_FILE = " << tour_path << "\n";
    par_file << "RUNS = 3\n";
    par_file << "TRACE_LEVEL = 0\n";
    par_file << "SEED = " << std::time(nullptr) << "\n";
    par_file.close();

    // 3. 调用LKH_main函数（保持不变）
    char* args[] = {
        (char*)"LKH",
        (char*)par_path.c_str()
    };
    int argc = 2;
    int result = LKH_main(argc, args);
    if (result != 0) {
        std::cerr << "LKH优化失败，错误码: " << result << std::endl;
        return;
    }

    // 4. 读取优化后的环（保持不变）
    std::ifstream tour_file(tour_path);
    if (!tour_file) {
        std::cerr << "无法打开结果文件: " << tour_path << std::endl;
        return;
    }

    std::string line;
    std::vector<int> new_ring_order;
    bool in_tour_section = false;

    while (std::getline(tour_file, line)) {
        if (line.find("TOUR_SECTION") != std::string::npos) {
            in_tour_section = true;
            continue;
        }
        if (in_tour_section) {
            if (line.find("-1") != std::string::npos) break;

            std::istringstream iss(line);
            int node_index;
            while (iss >> node_index) {
                if (node_index > 0 && node_index <= s.ring.size()) {
                    new_ring_order.push_back(s.ring[node_index - 1]);
                }
            }
        }
    }
    tour_file.close();

    // 5. 更新解的环结构（保持不变）
    if (new_ring_order.size() == s.ring.size()) {
        s.ring = new_ring_order;
        s.routing_cost = 0;
        for (int i = 0; i < s.ring.size(); ++i) {
            int j = (i + 1) % s.ring.size();
            s.routing_cost += alpha * graph.routing_cost[s.ring[i]][s.ring[j]];
        }

        // 计算并记录优化效果
        double new_cost = s.total_cost();
        double improvement = original_cost - new_cost;
        lkh_call_count++;
        lkh_total_improvement += improvement;

        if (improvement > 0) {
            cout << "LKH优化提升: " << improvement << endl;
        }
    }
    else {
        std::cerr << "LKH返回的环大小不一致: " << new_ring_order.size()
            << " (应为 " << s.ring.size() << ")" << std::endl;
    }

    
}

bool AMNSSolver::validate_solution2( Solution& s) {
    // 检查环大小至少为3
    if (s.ring.size() < 3) {
        std::cerr << "错误: 环大小小于3 (" << s.ring.size() << ")。" << std::endl;
        return false;
    }

    // 检查depot是否在环上
    if (std::find(s.ring.begin(), s.ring.end(), graph.depot) == s.ring.end()) {
        std::cerr << "错误: Depot " << graph.depot << " 不在环上。" << std::endl;
        return false;
    }

    // 检查环节点唯一性
    std::unordered_set<int> ring_set(s.ring.begin(), s.ring.end());
    if (ring_set.size() != s.ring.size()) {
        std::cerr << "错误: 环中包含重复节点。" << std::endl;
        return false;
    }

    // 检查in_ring和ring列表的一致性
    for (int v : s.ring) {
        if (v < 0 || v >= graph.nodes.size() || !s.in_ring[v]) {
            std::cerr << "错误: 节点 " << v << " 在环列表中但未标记为 in_ring。" << std::endl;
            return false;
        }
    }

    // 检查非环节点的分配目标在环上且有效
    for (int u = 0; u < graph.nodes.size(); ++u) {
        if (!s.in_ring[u]) {
            int assigned_to = s.assignments[u];

            // 检查分配目标是否有效
            if (assigned_to < 0 || assigned_to >= graph.nodes.size()) {
                std::cerr << "错误: 非环节点 " << u << " 被分配到无效节点 " << assigned_to << "（越界）。" << std::endl;
                return false;
            }

            // 检查分配目标是否在环上
            if (!s.in_ring[assigned_to]) {
                std::cerr << "错误: 非环节点 " << u << " 被分配到非环节点 " << assigned_to << "。" << std::endl;
                return false;
            }

            // 检查assignment_map中是否存在该分配关系
            if (!s.assignment_map.count(assigned_to)) {
                std::cerr << "错误: 分配映射中缺少节点 " << assigned_to << " 的条目（节点 " << u << " 应分配到此）。" << std::endl;
                return false;
            }

            // 检查assignment_map中是否包含节点u
            const auto& assign_list = s.assignment_map.at(assigned_to);
            if (std::find(assign_list.begin(), assign_list.end(), u) == assign_list.end()) {
                std::cerr << "错误: 非环节点 " << u << " 未在分配映射中节点 " << assigned_to << " 的列表中。" << std::endl;
                return false;
            }
        }
    }

    // 检查assignment_map的完整性：所有映射条目都应指向环上节点
    for (const auto& pair : s.assignment_map) {
        int ring_node = pair.first;
        if (!s.in_ring[ring_node]&& s.assignment_map[ring_node].size()!=0) {
            std::cerr << "错误: 分配映射中包含非环节点 " << ring_node << " 作为键。" << std::endl;
            return false;
        }

        for (int u : pair.second) {
            if (u < 0 || u >= graph.nodes.size()) {
                std::cerr << "错误: 分配映射中包含无效节点 " << u << " 在节点 " << ring_node << " 的列表中。" << std::endl;
                return false;
            }
            /*if (s.assignments[u] != ring_node) {
                std::cerr << "错误: 分配映射不一致 - 非环节点 " << u << " 在映射中分配到 " << ring_node << "，但 assignments 数组指向 " << s.assignments[u] << "。" << std::endl;
                return false;
            }*/
        }
    }

    // 所有检查通过，解有效
    return true;
}



// 带禁忌表的2-opt
Solution AMNSSolver::tabu_two_opt(Solution s, double original_cost) {
    bool improved;
    int iterations = 0;
    const int MAX_ITERATIONS = 100;

    do {
        improved = false;
        double best_delta = 0;
        int best_i = -1, best_j = -1;
        bool best_is_tabu = false;

        // 搜索所有可能的2-opt交换
        for (int i = 0; i < s.ring.size() - 1; ++i) {
            for (int j = i + 2; j < s.ring.size(); ++j) {
                int node_i = s.ring[i];
                int node_j = s.ring[j];

                // 检查是否禁忌
                bool is_tabu_move = is_tabu(node_i, node_j, 0);

                double delta = graph.routing_cost[s.ring[i]][s.ring[j]]
                    + graph.routing_cost[s.ring[i + 1]][s.ring[(j + 1) % s.ring.size()]]
                    - graph.routing_cost[s.ring[i]][s.ring[i + 1]]
                    - graph.routing_cost[s.ring[j]][s.ring[(j + 1) % s.ring.size()]];

                // 如果移动禁忌但满足藐视准则，仍可考虑
                bool can_accept = !is_tabu_move ||
                    aspiration_criterion(original_cost, s.total_cost() + alpha * delta);

                if (can_accept && delta < best_delta) {
                    best_delta = delta;
                    best_i = i;
                    best_j = j;
                    best_is_tabu = is_tabu_move;
                }
            }
        }

        if (best_delta < -1e-6) {
            // 执行交换
            reverse(s.ring.begin() + best_i + 1, s.ring.begin() + best_j + 1);
            s.routing_cost += alpha * best_delta;
            improved = true;

            // 记录禁忌移动（除非满足藐视准则）
            if (!best_is_tabu) {
                int node1 = s.ring[best_i];
                int node2 = s.ring[best_j];
                add_tabu_move(node1, node2, 0);
            }
        }

        iterations++;
    } while (improved && iterations < MAX_ITERATIONS);

    return s;
}

// 带禁忌表的节点删除
Solution AMNSSolver::tabu_drop_node(Solution s, double original_cost) {
    if (s.ring.size() <= 3) return s;

    Solution best_solution = s;
    double min_cost = s.total_cost();
    bool improvement_found = false;

    for (size_t pos = 1; pos < s.ring.size(); ++pos) {
        int v = s.ring[pos];
        if (v == graph.depot) continue;

        // 检查删除操作是否禁忌
        if (is_tabu(v, -1, 2)) { // 删除操作的禁忌检查
            if (!aspiration_criterion(original_cost, min_cost)) {
                continue; // 跳过禁忌移动
            }
        }

        Solution temp = s;
        remove_node(temp, v);//TODO 可优化增量
        if (temp.total_cost() < min_cost) {
            min_cost = temp.total_cost();
            best_solution = temp;
            improvement_found = true;

            // 记录禁忌移动：禁止在禁忌期内重新添加此节点
            if (!is_tabu(v, -1, 2)) { // 如果不是因为藐视准则接受的
                add_tabu_move(v, -1, 2); // 禁忌删除操作
                add_tabu_move(-1, v, 3, DEFAULT_TABU_TENURE / 2); // 较短禁忌期禁止添加
            }
        }
    }
    //if (!improvement_found) {
    //    // 尝试允许一些禁忌移动
    //    for (size_t pos = 1; pos < s.ring.size(); ++pos) {
    //        int v = s.ring[pos];
    //        if (v == graph.depot) continue;

    //        if (is_tabu(v, -1, 2)) { // 当前禁忌
    //            Solution temp = s;
    //            // ... 执行删除操作

    //            if (temp.total_cost() < min_cost * 0.98) { // 较大改进时打破禁忌
    //                min_cost = temp.total_cost();
    //                best_solution = temp;
    //                cout << "★ 打破禁忌删除节点 " << v << "，改进: "
    //                    << (s.total_cost() - min_cost) << endl;
    //                break;
    //            }
    //        }
    //    }
    //}
    return best_solution;
}

// 带禁忌表的节点添加
Solution AMNSSolver::tabu_add_node(Solution s, double original_cost) {
    Solution best_solution = s;
    double min_cost = s.total_cost();
    bool improvement_found = false;

    for (int v = 0; v < graph.nodes.size(); ++v) {
        if (!s.in_ring[v] && v != graph.depot) {
            // 检查添加操作是否禁忌
            if (is_tabu(-1, v, 3)) { // 添加操作的禁忌检查
                if (!aspiration_criterion(original_cost, min_cost)) {
                    continue;
                }
            }

            Solution temp = s;
            insert_node(temp, v);

            if (temp.total_cost() < min_cost) {
                min_cost = temp.total_cost();
                best_solution = temp;
                improvement_found = true;

                // 记录禁忌移动
                if (!is_tabu(-1, v, 3)) {
                    add_tabu_move(-1, v, 3); // 禁忌添加操作
                    add_tabu_move(v, -1, 2, DEFAULT_TABU_TENURE / 2); // 较短禁忌期禁止删除
                }
            }
        }
    }

    return best_solution;
}


// 修改后的局部搜索算子
Solution AMNSSolver::apply_tabu_local_search_operator(Solution s, int operator_index) {
    Solution result = s;
    if (!validate_solution2(result)) {
        cout << "无效初始解" << endl;
    }

    double original_cost = s.total_cost();

    switch (operator_index) {
    case 0:
        result = tabu_two_opt(s, original_cost);
        break;
    case 1:
		result = randomized_three_opt(s);
        break;
    case 2:
        result = tabu_add_node(s, original_cost);
        break;
    case 3:
        result = tabu_drop_node(s, original_cost);
        break;
    case 4:
        result = vertex_exchange(s);
        break;
    default:
        result = tabu_two_opt(s, original_cost);
    }
    return result;
}

// 修改后的顶点交换算子（带禁忌搜索和邻域限制）
Solution AMNSSolver::vertex_exchange(Solution s) {
    double original_cost = s.total_cost();
    Solution best_solution = s;
    double best_improvement = 0.0;
    bool improvement_found = false;

    // 初始化邻域缓存
    init_exchange_neighborhood(s);

    // 收集环上节点（排除depot）
    vector<int> ring_nodes;
    for (int i = 0; i < graph.nodes.size(); ++i) {
        if (s.in_ring[i] && i != graph.depot) {
            ring_nodes.push_back(i);
        }
    }

    if (ring_nodes.empty()) {
        cout << "  [顶点交换] 无可用环节点" << endl;
        return s;
    }

    int evaluated_pairs = 0;
    int improving_pairs = 0;
    int tabu_skipped = 0;

    // 使用邻域限制的遍历策略
    vector<tuple<double, int, int>> candidate_pairs; // (潜力评分, 环节点, 非环节点)

    // 构建候选对列表（带潜力评估）
    for (int vj : ring_nodes) {
        if (!ring_neighborhood_cache.count(vj)) continue;

        for (int vl : ring_neighborhood_cache[vj]) {
            // 计算交换潜力（基于距离启发式）
            double potential = calculate_exchange_potential(s, vj, vl);
            candidate_pairs.emplace_back(potential, vj, vl);
        }
    }

    // 按潜力排序（潜力越大越优先）
    sort(candidate_pairs.begin(), candidate_pairs.end(),
        [](const auto& a, const auto& b) { return get<0>(a) > get<0>(b); });

    // 限制评估数量
    int exchange_size = params.min_exchange_size;
    int max_evaluations = min(exchange_size, (int)candidate_pairs.size());


    // 评估候选对
    for (int i = 0; i < max_evaluations && i < candidate_pairs.size(); ++i) {
        int vj = get<1>(candidate_pairs[i]);
        int vl = get<2>(candidate_pairs[i]);

        evaluated_pairs++;

        // 禁忌检查 - 移动类型4表示顶点交换
        if (is_tabu(vj, vl, 4)) {
            // 检查是否满足藐视准则
            if (!aspiration_criterion(best_solution.total_cost(), original_cost)) {
                tabu_skipped++;
                continue;
            }
        }

        // 创建临时解进行交换操作
        Solution temp = s;
        double temp_original_cost = temp.total_cost();

        // 执行顶点交换：移除vj，插入vl
        bool success = perform_vertex_exchange(temp, vj, vl);

        if (!success) {
            continue;
        }

        // 重新评估解的成本
        evaluate(temp);

        // 计算交换后的成本
        double new_cost = temp.total_cost();
        double improvement = temp_original_cost - new_cost;

        if (improvement > best_improvement + 1e-6) {
            best_improvement = improvement;
            best_solution = temp;
            improvement_found = true;
            improving_pairs++;

            // 添加禁忌移动
            add_tabu_move(vj, vl, 4);


            // 早期终止：如果改进很大，可以提前终止
            if (improvement > original_cost * 0.03) { // 3%的显著改进
                break;
            }
        }
    }

    // 如果没有找到改进，尝试随机抽样一些额外的候选对
    if (!improvement_found && candidate_pairs.size() > max_evaluations) {
        cout << "  [顶点交换] 尝试随机抽样额外候选" << endl;

        vector<int> additional_indices;
        for (int i = max_evaluations; i < candidate_pairs.size(); i++) {
            additional_indices.push_back(i);
        }
        shuffle(additional_indices.begin(), additional_indices.end(), rng);

        int additional_trials = min(10, (int)additional_indices.size());
        for (int i = 0; i < additional_trials; i++) {
            int idx = additional_indices[i];
            int vj = get<1>(candidate_pairs[idx]);
            int vl = get<2>(candidate_pairs[idx]);

            // 禁忌检查
            if (is_tabu(vj, vl, 4) &&
                !aspiration_criterion(best_solution.total_cost(), original_cost)) {
                continue;
            }

            Solution temp = s;
            if (perform_vertex_exchange(temp, vj, vl)) {
                evaluate(temp);
                double improvement = s.total_cost() - temp.total_cost();

                if (improvement > best_improvement + 1e-6) {
                    best_improvement = improvement;
                    best_solution = temp;
                    improvement_found = true;
                    improving_pairs++;
                    add_tabu_move(vj, vl, 4);
                    break;
                }
            }
        }
    }

    cout << "  [顶点交换] 统计: 评估" << evaluated_pairs << "对, 改进" << improving_pairs
        << "对, 禁忌跳过" << tabu_skipped << "对, 最佳改进" << best_improvement << endl;

    if (improvement_found && best_improvement > 1e-6) {
        return best_solution;
    }
    else {
        return s;
    }
}


// 计算顶点交换的潜力评分
double AMNSSolver::calculate_exchange_potential(Solution& s, int vj, int vl) {
    double potential = 0.0;

    // 因素1: 路由成本改进潜力
    // 估计移除vj带来的路由成本节省
    auto vj_pos = find(s.ring.begin(), s.ring.end(), vj);
    if (vj_pos == s.ring.end()) return 0.0;

    int pos = distance(s.ring.begin(), vj_pos);
    int prev = s.ring[(pos - 1 + s.ring.size()) % s.ring.size()];
    int next = s.ring[(pos + 1) % s.ring.size()];

    double routing_saving = graph.routing_cost[prev][vj] + graph.routing_cost[vj][next]
        - graph.routing_cost[prev][next];

        // 因素2: 插入vl的额外成本估计
        double min_insert_cost = INF;
        for (int i = 0; i < s.ring.size(); ++i) {
            if (s.ring[i] == vj) continue; // 跳过要删除的位置

            int j = (i + 1) % s.ring.size();
            double insert_cost = graph.routing_cost[s.ring[i]][vl]
                + graph.routing_cost[vl][s.ring[j]]
                    - graph.routing_cost[s.ring[i]][s.ring[j]];
                min_insert_cost = min(min_insert_cost, insert_cost);
        }

        // 因素3: 分配成本改进潜力
        double assign_potential = 0.0;

        // vl加入环后可能改善分配
        if (s.assignment_map.count(vj)) {
            for (int u : s.assignment_map.at(vj)) {
                // 如果vl比当前分配更近，有改进潜力
                if (graph.assign_cost[u][vl] < graph.assign_cost[u][s.assignments[u]]) {
                    assign_potential += graph.assign_cost[u][s.assignments[u]] - graph.assign_cost[u][vl];
                }
            }
        }

        // 综合潜力评分（加权和）
        potential = alpha * routing_saving - alpha * min_insert_cost + (10 - alpha) * assign_potential;

        return potential;
}


//节点重要性 new 
vector<NodeImportance> AMNSSolver::evaluate_node_importance( Solution& s) {
    vector<NodeImportance> importance_list;

    for (int v : s.ring) {
        if (v == graph.depot) continue; // 不评估根节点

        NodeImportance ni;
        ni.node_id = v;

        // 1. 计算节点对环路由成本的贡献
        auto pos = find(s.ring.begin(), s.ring.end(), v);
        int idx = distance(s.ring.begin(), pos);
        int prev = s.ring[(idx - 1 + s.ring.size()) % s.ring.size()];
        int next = s.ring[(idx + 1) % s.ring.size()];

        // 如果删除v，路由成本的变化（负值表示成本减少）
        double routing_saving = graph.routing_cost[prev][v] + graph.routing_cost[v][next]
            - graph.routing_cost[prev][next];
            ni.routing_contribution = routing_saving;

            // 2. 计算节点对分配成本的贡献
            double assignment_impact = 0.0;
            if (s.assignment_map.count(v)) {
                for (int u : s.assignment_map.at(v)) {
                    // 找到v被删除后，u的最佳替代分配
                    int best_alternative = find_best_alternative_assignment(s, u, v);
                    double new_cost = graph.assign_cost[u][best_alternative];
                    double old_cost = graph.assign_cost[u][v];
                    assignment_impact += (new_cost - old_cost); // 正值表示成本增加
                }
            }
            ni.assignment_benefit = -assignment_impact; // 转换为收益形式

            // 3. 综合评分：路由节省 - 分配成本增加（考虑权重alpha）
            ni.importance_score = alpha * ni.routing_contribution
                + (10 - alpha) * ni.assignment_benefit;

            importance_list.push_back(ni);
    }

    // 按重要性评分排序（分数越低表示越应该删除）
    sort(importance_list.begin(), importance_list.end(),
        [](const NodeImportance& a, const NodeImportance& b) {
            return a.importance_score < b.importance_score;
        });

    return importance_list;
}

// 找到删除v后，u的最佳替代分配节点
int AMNSSolver::find_best_alternative_assignment( Solution& s, int u, int excluded_node) {
    int best_assign = -1;
    double min_cost = INF;

    for (int v : s.ring) {
        if (v != excluded_node && v != u) { // 不能分配到被删除的节点或自身
            double cost = graph.assign_cost[u][v];
            if (cost < min_cost) {
                min_cost = cost;
                best_assign = v;
            }
        }
    }

    return (best_assign != -1) ? best_assign : graph.depot;
}


// 批量删除
Solution AMNSSolver::batch_intelligent_removal(Solution s, int max_removals) {
    Solution current = s;
    bool improved;
    int removals = 0;

    do {
        improved = false;
        auto importance_list = evaluate_node_importance(current);

        // 尝试删除最不重要的节点
        for (const auto& ni : importance_list) {
            if (removals >= max_removals || current.ring.size() <= 3) break;

            Solution temp = current;
            double old_cost = temp.total_cost();

            remove_node(temp, ni.node_id);
			evaluate(temp);
            if (temp.total_cost() < old_cost) {//temp.total_cost() < old_cost
                current = temp;
                improved = true;
                removals++;

                break; // 重新评估重要性
            }
        }
    } while (improved && removals < max_removals);

    return current;
}


// 初始化模拟退火
void AMNSSolver::initialize_simulated_annealing() {
    current_temperature = params.initial_temperature;
    sa_iteration_count = 0;
    cout << "模拟退火初始化: 初始温度 = " << current_temperature << endl;
}

// 计算接受恶化解的概率
double AMNSSolver::calculate_acceptance_probability(double delta_cost, double temperature) {
    if (delta_cost < 0) {
        return 1.0; // 改进解总是接受
    }
    // 使用经典的Metropolis准则
    return exp(-delta_cost / temperature);
}

// 判断是否接受恶化解
bool AMNSSolver::accept_worse_solution(double current_cost, double new_cost, double temperature) {
    double delta_cost = new_cost - current_cost;

    if (delta_cost < -1e-6) {
        return true; // 总是接受改进解
    }

    double acceptance_prob = calculate_acceptance_probability(delta_cost, temperature);
    double random_val = (double)rand() / RAND_MAX;

    return random_val < acceptance_prob;
}

// 更新温度（几何冷却方案）
void AMNSSolver::update_temperature() {
    sa_iteration_count++;

    // 每个温度进行一定次数的迭代后降温
    if (sa_iteration_count % params.sa_iterations_per_temp == 0) {
        current_temperature *= params.cooling_rate;

        // 确保温度不低于最小值
        if (current_temperature < params.final_temperature) {
            current_temperature = params.final_temperature;
        }

        cout << "温度更新: " << current_temperature
            << ", 迭代次数: " << sa_iteration_count << endl;
    }
}

Solution AMNSSolver::diversify_solution( Solution& best) {
    cout << "执行解多样化策略..." << endl;

    Solution diversified = best;

    // 策略1: 随机删除一部分环节点（20%-40%）
    int min_ring_size = 3;
    double removal_ratio = 0.2 + 0.2 * (rand() / (double)RAND_MAX);
    int removal_count = max(1, static_cast<int>(best.ring.size() * removal_ratio));
    removal_count = min(removal_count, static_cast<int>(best.ring.size()) - min_ring_size);

    vector<int> removable_nodes;
    for (int v : best.ring) {
        if (v != graph.depot) removable_nodes.push_back(v);
    }
    shuffle(removable_nodes.begin(), removable_nodes.end(), rng);

    for (int i = 0; i < removal_count; ++i) {
        remove_node(diversified, removable_nodes[i]);
    }

    // 策略2: 随机添加一些非环节点
    int add_count = removal_count / 2;
    vector<int> addable_nodes;
    for (int v = 0; v < graph.nodes.size(); ++v) {
        if (!diversified.in_ring[v] && v != graph.depot) {
            addable_nodes.push_back(v);
        }
    }
    shuffle(addable_nodes.begin(), addable_nodes.end(), rng);

    for (int i = 0; i < min(add_count, (int)addable_nodes.size()); ++i) {
        insert_node(diversified, addable_nodes[i]);
    }

    // 重新评估解
    evaluate(diversified);

    cout << "多样化完成: 删除" << removal_count << "节点, 添加"
        << min(add_count, (int)addable_nodes.size()) << "节点"
        << ", 新成本: " << diversified.total_cost() << endl;

    return diversified;
}

// 针对不同问题规模的推荐参数
void AMNSSolver::set_SA_parameters_based_on_problem_size() {
    int n = graph.nodes.size();

    if (n < 100) {
        // 小规模问题
        params.initial_temperature = 500;
        params.cooling_rate = 0.9;
        params.sa_iterations_per_temp = 5;
    }
    else if (n < 150) {
        // 中等规模问题
        params.initial_temperature = 1000;
        params.cooling_rate = 0.95;
        params.sa_iterations_per_temp = 10;
    }
    else {
        // 大规模问题
        params.initial_temperature = 2000;
        params.cooling_rate = 0.98;
        params.sa_iterations_per_temp = 15;
    }

    cout << "SA参数设置: 温度=" << params.initial_temperature
        << ", 冷却率=" << params.cooling_rate
        << ", 迭代次数=" << params.sa_iterations_per_temp << endl;
}
