#pragma once
#include "rsp_graph.h"
#include <string>

#ifdef __cplusplus
extern "C" {
#endif
	// 声明 LKH 主函数（需在 LKH.c 中修改为可调用的接口）
	void LKH_Solver(int argc, char* argv[]);

#ifdef __cplusplus
}
#endif

Solution solve_with_lkh(const RSPGraph& graph, const std::string& input_dir, const std::string& output_dir, const std::string tspfilename);