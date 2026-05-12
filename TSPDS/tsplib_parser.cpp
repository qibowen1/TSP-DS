#include "tsplib_parser.h"
#include "solver.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

namespace fs = std::filesystem;
using namespace std;

RSPGraph parseTSPLIB(const string& filename) {
    ifstream file(filename);
    if (!file) {
        cerr << "无法打开文件: " << filename << endl;
        exit(1);
    }

    RSPGraph graph;
    string line;
    int dimension = 0;
    bool node_section = false;

    while (getline(file, line)) {
        // 清理行首尾空格
        line.erase(line.find_last_not_of(" \t") + 1);
        line.erase(0, line.find_first_not_of(" \t"));

        // 统一转为小写处理
        string lowerLine = line;
        transform(lowerLine.begin(), lowerLine.end(), lowerLine.begin(), ::tolower);

        if (lowerLine.find("dimension") != string::npos) {
            size_t colon = line.find(':');
            if (colon != string::npos) {
                string dimStr = line.substr(colon + 1);
                // 过滤非数字字符
                dimStr.erase(remove_if(dimStr.begin(), dimStr.end(),
                    [](char c) { return !isdigit(c); }), dimStr.end());
                if (!dimStr.empty()) {
                    dimension = stoi(dimStr);
                    graph.initialize(dimension);
                    graph.nodes.resize(dimension);
                }
            }
        }
        else if (lowerLine.find("node_coord_section") != string::npos) {
            node_section = true;
            continue; // 跳过该行
        }
        else if (line.find("EOF") != string::npos) {
            break;
        }

        if (node_section) {
            istringstream iss(line);
            int id;
            double x, y;
            iss >> id >> x >> y;
            id--; // 转换为0-based索引
            graph.nodes[id] = { x, y };
        }
    }

    // 计算距离矩阵
    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension; ++j) {
            double dx = graph.nodes[i].first - graph.nodes[j].first;
            double dy = graph.nodes[i].second - graph.nodes[j].second;
            double dist = hypot(dx, dy);
            // 修改为四舍五入后的整数距离
            int dist_int = static_cast<int>(round(dist));  // 添加四舍五入转换[1,3](@ref)

            // 更新距离矩阵
            graph.routing_cost[i][j] = dist_int;
            graph.assign_cost[i][j] = dist_int;
        }
    }

    return graph;
}


