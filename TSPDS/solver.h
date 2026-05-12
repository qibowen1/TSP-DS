#pragma once
#include "rsp_graph.h"
#include "param.h"
#include <memory>
#include <string>
#include <algorithm>

using namespace std;


class AMNSSolver {

public:
	AlgorithmParams params;
	Solution auto_tune(int benchmark_opt);
	
private:
	RSPGraph graph;
	int alpha;
	string filename;
	Solution best;
	int glo_benchmark_opt;

	int lkh_call_count = 0;
	double lkh_total_improvement = 0.0;


	unordered_map<int, vector<int>> static_near_cache; // 静态缓存：节点到所有节点的最近距离排序

	unordered_map<int, vector<int>> static_near_cache_closet; // 静态缓存：

	// 邻域限制参数
	const int NEIGHBORHOOD_SIZE = params.NEIGHBORHOOD_SIZE;  // 每个环节点考虑的非环节点数量
	unordered_map<int, vector<int>> ring_neighborhood_cache;  // 环节点的邻域缓存

	// 简单参数配置
	struct SimpleConfig {
		string name;
		double rcl;
		int k_max;
		double max_time;
	};

	vector<SimpleConfig> configs = {
		{"1型", 0.7, 1, 120.0},
		{"2型", 0.9, 2, 60.0},
		{"3型", 0.95, 3, 30.0}
	};

	// 初始化邻域缓存
    void init_exchange_neighborhood(const Solution& s) {
        ring_neighborhood_cache.clear();
        
        // 收集环节点和非环节点
        vector<int> ring_nodes;
        vector<int> non_ring_nodes;
        
        for (int i = 0; i < graph.nodes.size(); ++i) {
            if (s.in_ring[i] && i != graph.depot) {
                ring_nodes.push_back(i);
            } else if (!s.in_ring[i]) {
                non_ring_nodes.push_back(i);
            }
        }
        
        // 为每个环节点构建邻域
        for (int ring_node : ring_nodes) {
            vector<pair<double, int>> candidates; // (距离, 非环节点)
            
            for (int non_ring : non_ring_nodes) {
                // 使用路由成本作为距离度量
                double distance = graph.routing_cost[ring_node][non_ring];
                candidates.emplace_back(distance, non_ring);
            }
            
            // 按距离排序，取前NEIGHBORHOOD_SIZE个
            sort(candidates.begin(), candidates.end());
            vector<int> neighborhood;
            for (int i = 0; i < min(NEIGHBORHOOD_SIZE, (int)candidates.size()); ++i) {
                neighborhood.push_back(candidates[i].second);
            }
            
            ring_neighborhood_cache[ring_node] = neighborhood;
        }
    }


public:
	AMNSSolver(const RSPGraph& g, int a,
		std::string filename);
	void solve(int benchmark_opt, int MAX_ITER);
	Solution getBestSolution() ;


	//统计
private:
	vector<int> operator_calls;    // 算子调用次数
	vector<int> operator_improves; // 算子改进次数
	vector<string> operator_names; // 算子名称

	void init_simple_stats() {
		operator_names = { "2-opt", "3-opt", "drop_node", "add_node", "vertex_exchange" };
		operator_calls = vector<int>(operator_names.size(), 0);
		operator_improves = vector<int>(operator_names.size(), 0);
	}


private:
	
	Solution greedy_construction_all_nodes();
	// LKH优化函数
	void optimize_ring_with_lkh(Solution& s);

	Solution variable_neighborhood_VNS3_SA(Solution s, int benchmark_opt);
	void build_static_cache_cloest();
	//初始解

	Solution sample_greedy_construction(double custom_rcl) ;

	// 新增方法


	//localsearch


	Solution variable_neighborhood_VNS3(Solution s, int benchmark_opt);



	Solution local_search_FULL_VND(Solution s) ;//完全遍历


	//遍历领域所有解
	Solution exhaustive_add_node(Solution s) ;

	Solution exhaustive_drop_node(Solution s) ;

	Solution exhaustive_two_opt(Solution s) ;

	Solution cluster_insertion_operator(Solution s);

	double calculate_insertion_cost(int prev, int next, const vector<int>& seq);

	void improved_remove_node(Solution& s, int node_to_remove);

	Solution swap_ring_node_with_assignment(Solution s);

	// 在AMNSSolver类中添加

	Solution randomized_three_opt(Solution s) ;



	//tools

	int find_closest_ring_node(const Solution& s, int u) ;



	void insert_node(Solution& s, int v) ;


	void evaluate(Solution& s) ;

	void reallocateNoCircle(Solution& s) ;

	bool isSolutionValid(const Solution& s, const RSPGraph& graph);

	//shaking

	Solution random_node_swap(Solution s, int k);
	Solution multiple_node_operation(Solution s, int k);
	Solution assignment_perturbation(Solution s, int k);
	void remove_node(Solution& s, int v);
	void reallocate_affected_nodes(Solution& s, int removed_node);
	Solution remove_most_assigned_nodes(Solution s, int k);
	//adaptive

	Solution apply_shaking_operator(Solution s, int operator_index, int k);
	Solution apply_local_search_operator(Solution s, int operator_index);

	bool validate_solution2( Solution& s);

	Solution vertex_exchange(Solution s);
	bool perform_vertex_exchange(Solution& s, int vj, int vl);



	Solution tabu_two_opt(Solution s, double original_cost);
	Solution tabu_drop_node(Solution s, double original_cost);
	Solution tabu_add_node(Solution s, double original_cost);
	Solution apply_tabu_local_search_operator(Solution s, int operator_index);
	double calculate_exchange_potential( Solution& s, int vj, int vl);
	private:
		// 禁忌表相关成员
		struct TabuMove {
			int node1;           // 操作涉及的第一个节点(环)
			int node2;           // 操作涉及的第二个节点（非环）
			int move_type;       // 移动类型：0=2-opt, 1=3-opt, 2=删除, 3=添加, 4=交换
			int tabu_tenure;     // 禁忌期限
			int iteration;       // 执行迭代
		};

		vector<TabuMove> tabu_list;                    // 禁忌表
		int current_iteration = 0;                     // 当前迭代计数
		const int DEFAULT_TABU_TENURE = params.DEFAULT_TABU_TENURE;            // 默认禁忌期限
		const int MAX_TABU_SIZE = params.MAX_TABU_SIZE;                // 禁忌表最大大小

		// 检查移动是否在禁忌表中
		bool is_tabu(int node1, int node2, int move_type) {
			// 清理过期禁忌
			tabu_list.erase(
				remove_if(tabu_list.begin(), tabu_list.end(),
					[&](const TabuMove& move) {
						return (current_iteration - move.iteration) > move.tabu_tenure;
					}),
				tabu_list.end()
			);

			// 检查是否禁忌
			for (const auto& move : tabu_list) {
				if (move.move_type == move_type) {
					if ((move.node1 == node1 && move.node2 == node2) ||
						(move.node1 == node2 && move.node2 == node1)) {
						return true;
					}
				}
			}
			return false;
		}

		// 添加禁忌移动
		void add_tabu_move(int node1, int node2, int move_type, int tenure = -1) {
			if (tenure == -1) tenure = DEFAULT_TABU_TENURE;

			// 限制禁忌表大小
			if (tabu_list.size() >= MAX_TABU_SIZE) {
				tabu_list.erase(tabu_list.begin());
			}

			TabuMove move{ node1, node2, move_type, tenure, current_iteration };
			tabu_list.push_back(move);
		}

		// 藐视准则：如果改进足够大，允许禁忌移动
		bool aspiration_criterion(double current_best, double candidate_cost, double improvement_threshold = 0.05) {
			return candidate_cost < current_best * (1 - improvement_threshold);
		}

		//智能删除
		vector<NodeImportance> evaluate_node_importance( Solution& s);
		int find_best_alternative_assignment(Solution& s, int u, int excluded_node);
		Solution batch_intelligent_removal(Solution s, int max_removals);


		// 环上被分配少的节点，识别并删除未被分配的环节点
		Solution remove_unassigned_ring_nodes(Solution s);
		double estimate_deletion_gain(const Solution& s, int v);
		Solution batch_remove_unassigned_nodes(Solution s, int max_removals);
		vector<int> find_all_unassigned_ring_nodes( Solution& s);
		Solution balanced_unassigned_removal(Solution s);
		double calculate_routing_impact( Solution& s, int v);
		Solution periodic_unassigned_cleanup(Solution s);

		// 在 AMNSSolver 类的 public 部分添加：
		Solution batch_ring_node_swap(Solution s, int batch_size = 5);
		Solution batch_remove_isolated_ring_nodes(Solution s, int max_removals = 10);

		//聚类初始化
		Solution cluster_based_initialization();
		int calculate_optimal_clusters();
		vector<vector<int>> kmeans_clustering(int k);
		vector<int> select_ring_nodes(const vector<vector<int>>& clusters);
		void construct_ring(Solution& s, vector<int>& ring_nodes);
		void sort_ring_nodes(vector<int>& ring_nodes);
		void assign_non_ring_nodes(Solution& s, const vector<int>& ring_nodes);
		double euclidean_distance(pair<double, double> a, pair<double, double> b);
		void add_extra_ring_nodes(vector<int>& ring_nodes,
			const vector<vector<int>>& clusters);

		// 辅助方法
private:
	vector<pair<int, vector<int>>> get_ring_nodes_with_assignments( Solution& s);
	
	double evaluate_swap_gain( Solution& s, int ring_node, int candidate_node);
	vector<int> find_isolated_ring_nodes(Solution& s);
	double estimate_isolated_removal_gain(Solution& s, int v);

	// 在 AMNSSolver 类的 public 部分添加：
Solution batch_insert_non_ring_nodes(Solution s, int batch_size = 8);

// 辅助方法
private:
vector<pair<double, int>> evaluate_insertion_potential( Solution& s);
double calculate_insertion_gain( Solution& s, int node);
vector<int> select_promising_non_ring_nodes( Solution& s, int target_count);
void smart_batch_insertion(Solution& s,  vector<int>& nodes_to_insert);



public:
	// 更新迭代计数
	void increment_iteration() { current_iteration++; }

	// 清空禁忌表
	void clear_tabu() { tabu_list.clear(); }

	
	private:
		// 现有成员变量...

		// 模拟退火状态变量
		double current_temperature;
		int sa_iteration_count;
		Solution current_best_SA; // SA过程中的当前最优解

		// SA相关方法
		void initialize_simulated_annealing();
		bool accept_worse_solution(double current_cost, double new_cost, double temperature);
		void update_temperature();
		double calculate_acceptance_probability(double delta_cost, double temperature);

		Solution diversify_solution(Solution& best);
		void set_SA_parameters_based_on_problem_size();
};