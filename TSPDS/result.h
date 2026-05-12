#pragma once
#include <string>
struct Result {
    std::string filename;
    int alpha;
    double total_cost;
    double routing_cost;
    double assign_cost;
    double duration_s;
	double offset;//与基准的偏差

    // 添加构造函数进行四舍五入
    Result(std::string f, int a, double t, double r, double as, double d, double o)
        : filename(f), alpha(a),
        total_cost(round(t * 100) / 100),  // 保留2位小数
        routing_cost(round(r * 100) / 100),
        assign_cost(round(as * 100) / 100),
        duration_s(round(d * 100) / 100),
        offset(round(o * 100) / 100) {
    }
};
