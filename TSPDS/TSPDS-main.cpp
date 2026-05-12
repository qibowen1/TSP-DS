// main.cpp (or TSPDS-main.cpp) —— 已改为 Linux/Windows 通用相对路径版本
#include "TSPDSSolver.h"
#include "CplexF2Solver.h"
#include "CplexBendersSolver.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <map>
#include <mutex>
#include <execution>
#include <unordered_set>
#include <cmath>
#include <stdexcept>
#include <random>
#include <omp.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>

#include <cstdint>

using namespace std;
namespace fs = std::filesystem;


// -------- CSV 实例解析：id,x,y,flag(0/1/2) --------
struct CsvNodeRec {
    int id = -1;
    double x = 0.0;
    double y = 0.0;
    int flag = 0; // 0 eligible, 1 truck-only, 2 station
};

// --- CSV 安全输出：把含逗号/引号/换行的字段用双引号包起来，并把内部引号变成 "" ---
static std::string csvEscape(std::string s) {
    bool needQuote = false;
    for (char c : s) {
        if (c == '"') needQuote = true;
        if (c == ',' || c == '\n' || c == '\r') needQuote = true;
    }
    // 把 " 替换成 ""
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        if (c == '"') out += "\"\"";
        else out += c;
    }
    if (needQuote) return "\"" + out + "\"";
    return out;
}

static inline std::string trimCopy(std::string s) {
    auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
    return s;
}

static bool parseCsvLineLast4(const std::string& line, CsvNodeRec& out) {
    // 支持：
    // 4列: id,x,y,flag
    // 5列+: ...,id,x,y,flag（取最后4列）
    std::vector<std::string> cells;
    cells.reserve(8);

    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        cells.push_back(trimCopy(cell));
    }
    if ((int)cells.size() < 4) return false;

    // 取最后4个
    int base = (int)cells.size() - 4;
    try {
        out.id = std::stoi(cells[base + 0]);
        out.x = std::stod(cells[base + 1]);
        out.y = std::stod(cells[base + 2]);
        out.flag = std::stoi(cells[base + 3]);
        return true;
    }
    catch (...) {
        return false;
    }
}

static inline bool nearEq(double a, double b, double eps = 1e-9) {
    return std::fabs(a - b) <= eps;
}

static inline bool samePoint(const std::pair<double, double>& a,
    const std::pair<double, double>& b,
    double eps = 1e-9) {
    return nearEq(a.first, b.first, eps) && nearEq(a.second, b.second, eps);
}

#include <cctype>

// 解析一行 CSV，支持 "..." 以及 "" 转义
static std::vector<std::string> splitCsvRow(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    bool inQuotes = false;

    for (size_t i = 0; i < s.size(); ++i) {
        char c = s[i];

        if (inQuotes) {
            if (c == '"') {
                if (i + 1 < s.size() && s[i + 1] == '"') { // "" -> "
                    cur.push_back('"');
                    ++i;
                }
                else {
                    inQuotes = false;
                }
            }
            else {
                cur.push_back(c);
            }
        }
        else {
            if (c == '"') {
                inQuotes = true;
            }
            else if (c == ',') {
                out.push_back(cur);
                cur.clear();
            }
            else {
                cur.push_back(c);
            }
        }
    }
    out.push_back(cur);
    return out;
}

// 把 cells 拼回 CSV 一行（对每个 cell 做 csvEscape）
static std::string joinCsvRow(const std::vector<std::string>& cells) {
    std::string line;
    for (size_t i = 0; i < cells.size(); ++i) {
        if (i) line += ",";
        line += csvEscape(cells[i]);
    }
    return line;
}


// 从 *_0_80.csv / *_L_80.csv 构图
TSPDSGraph parseOriginCsvFile(
    const std::string& csvPath,
    int drone_count,
    double speed_ratio,
    double drone_range
) {
    std::ifstream in(csvPath);
    if (!in.is_open()) {
        throw std::runtime_error("Cannot open csv: " + csvPath);
    }

    std::vector<CsvNodeRec> recs;
    std::string line;
    while (std::getline(in, line)) {
        line = trimCopy(line);
        if (line.empty()) continue;

        CsvNodeRec r;
        if (!parseCsvLineLast4(line, r)) continue;
        if (r.id < 0) continue;
        recs.push_back(r);
    }
    in.close();

    if (recs.empty()) {
        throw std::runtime_error("CSV has no valid rows: " + csvPath);
    }

    // 先按 id 建表（假设 id 大概率是 0..N-1）
    int maxId = -1;
    for (auto& r : recs) maxId = std::max(maxId, r.id);
    int Nraw = maxId + 1;

    std::vector<std::pair<double, double>> coords(Nraw, { 0.0, 0.0 });
    std::vector<int> flags(Nraw, 0);
    std::vector<char> seen(Nraw, 0);

    for (auto& r : recs) {
        if (r.id >= 0 && r.id < Nraw) {
            coords[r.id] = { r.x, r.y };
            flags[r.id] = r.flag;
            seen[r.id] = 1;
        }
    }

    // depot 默认认为 id=0
    int depot = 0;
    if (depot < 0 || depot >= Nraw || !seen[depot]) {
        // 兜底：找最小出现过的 id
        depot = -1;
        for (int i = 0; i < Nraw; ++i) if (seen[i]) { depot = i; break; }
        if (depot < 0) throw std::runtime_error("Cannot find depot id in csv: " + csvPath);
    }

    // 删除 endDepot duplicate：坐标与 depot 完全一样、且 id != depot 的点（常见最后一个点）
    int dup = -1;
    for (int i = 0; i < Nraw; ++i) {
        if (!seen[i]) continue;
        if (i == depot) continue;
        if (samePoint(coords[i], coords[depot])) {
            // 若多个重复，通常取最大的那个（比如 N-1）
            if (dup < 0 || i > dup) dup = i;
        }
    }

    // 构造 keep 列表，并重映射 old->new（保证 depot 最终是 0）
    std::vector<int> keepOld;
    keepOld.reserve(Nraw);
    for (int i = 0; i < Nraw; ++i) {
        if (!seen[i]) continue;
        if (i == dup) continue;
        keepOld.push_back(i);
    }

    // 让 depot 放到 new=0
    auto itDep = std::find(keepOld.begin(), keepOld.end(), depot);
    if (itDep == keepOld.end()) throw std::runtime_error("Depot removed unexpectedly: " + csvPath);
    std::iter_swap(keepOld.begin(), itDep);

    std::vector<int> old2new(Nraw, -1);
    for (int newId = 0; newId < (int)keepOld.size(); ++newId) {
        old2new[keepOld[newId]] = newId;
    }

    int N = (int)keepOld.size();
    TSPDSGraph g;
    g.initialize(N);
    g.nodes.resize(N);
    g.depot = 0;
    g.drone_count = drone_count;
    g.speed_ratio = speed_ratio;
    g.drone_range = drone_range;

    // 填充 nodes + flags
    std::vector<int> newFlag(N, 0);
    for (int newId = 0; newId < N; ++newId) {
        int oldId = keepOld[newId];
        g.nodes[newId] = coords[oldId];
        newFlag[newId] = flags[oldId];
    }

    // 找 station（flag==2）
    int station = -1;
    for (int i = 0; i < N; ++i) {
        if (newFlag[i] == 2) { station = i; break; }
    }
    if (station < 0) {
        throw std::runtime_error("No drone station (flag=2) in csv: " + csvPath);
    }
    g.drone_station = station;

    // 初始化距离矩阵（用你原来的：truck=manhattan int, drone=euc int * speed_ratio）
    g.initDistanceMatrices(g.nodes, speed_ratio);

    // 设定类型
    g.is_drone_station.assign(N, false);
    g.is_drone_station[station] = true;

    g.is_drone_eligible.assign(N, false);
    g.is_truck_only.assign(N, true); // 默认 truck-only

    // depot / station 固定 truck-only, not eligible
    g.is_truck_only[g.depot] = true;
    g.is_truck_only[g.drone_station] = true;
    g.is_drone_eligible[g.depot] = false;
    g.is_drone_eligible[g.drone_station] = false;

    for (int i = 0; i < N; ++i) {
        if (i == g.depot) continue;
        if (i == g.drone_station) continue;

        if (newFlag[i] == 0) {
            g.is_drone_eligible[i] = true;
            g.is_truck_only[i] = false;
        }
        else if (newFlag[i] == 1) {
            g.is_drone_eligible[i] = false;
            g.is_truck_only[i] = true;
        }
        else if (newFlag[i] == 2) {
            // 已处理
        }
        else {
            // 未知 flag：按 truck-only 处理更安全
            g.is_drone_eligible[i] = false;
            g.is_truck_only[i] = true;
        }
    }

    return g;
}

static std::string depotTokenToPos(std::string t) {
    for (auto& c : t) c = (char)std::toupper((unsigned char)c);
    if (t == "0" || t == "C") return "C";
    if (t == "L") return "L";
    return t;
}

static void parseCsvName(const fs::path& p, std::string& instance_name, std::string& depot_pos, int& K) {
    std::string stem = p.stem().string(); // eil101_0_80
    std::vector<std::string> parts;
    {
        std::stringstream ss(stem);
        std::string tmp;
        while (std::getline(ss, tmp, '_')) parts.push_back(tmp);
    }
    instance_name = parts.empty() ? stem : parts[0];
    depot_pos = (parts.size() >= 2 ? depotTokenToPos(parts[1]) : "C");
    K = (parts.size() >= 3 ? std::stoi(parts[2]) : 80);
}

// ========= 全局：一次运行固定的 eligibility master seed（fork 前设置） =========
static uint32_t g_elig_master_seed = 0;
using namespace std;
namespace fs = std::filesystem;

// 64-bit FNV-1a：跨平台稳定（不要用 std::hash，可能不同实现不稳定）
static inline uint64_t fnv1a64(const std::string& s) {
    uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : s) {
        h ^= (uint64_t)c;
        h *= 1099511628211ULL;
    }
    return h;
}

// 64 -> 32 混合一下，作为 mt19937 seed
static inline uint32_t mix32(uint64_t x) {
    x ^= x >> 33;
    x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33;
    x *= 0xc4ceb9fe1a85ec53ULL;
    x ^= x >> 33;
    return (uint32_t)(x & 0xFFFFFFFFu);
}

struct ExactResult {
    bool ok = false;
    double makespan = -1;
    std::string status = "Unknown";

    double Tt = -1, Ta = -1, Td = -1;
    double best_integer = 0, best_bound = 0, mip_gap = 0;
    int find_best_time_sec = 0;

    bool feasible_solution = false;
    std::string feasible_info = "";

    // 新增
    int cplex_status = -1;
    std::string cplex_status_name = "Unknown";
    std::string cplex_result_type = "NOT_RUN";
};

static inline std::string exactKey(const std::string& tsp_path,
    const std::string& depot,
    int drone_count)
{
    std::string instance_base = fs::path(tsp_path).stem().string();
    return instance_base + "_" + depot + "_" + std::to_string(drone_count);
}


// 数据结构存储最优解信息
struct OptimalSolution {
    string instance_name;
    string depot_position;
    int drone_count;
    double optimal_makespan;
};

// 读取最优解文件
map<string, OptimalSolution> readOptimalSolutions(const string& filename) {
    map<string, OptimalSolution> optimalMap;
    ifstream file(filename);
    string line;

    if (!file.is_open()) {
        cerr << "Warning: Could not open optimal solutions file: " << filename << endl;
        return optimalMap;
    }

    cout << "Reading optimal solutions from: " << filename << endl;
    int count = 0;

    while (getline(file, line)) {
        if (line.empty()) continue;

        istringstream iss(line);
        string instance_depot;
        int drone_count;
        double optimal_value;

        if (iss >> instance_depot >> drone_count >> optimal_value) {
            // 解析实例名和仓库位置
            string instance_name, depot_position;

            // 查找最后一个下划线分隔实例名和仓库位置
            size_t underscore_pos = instance_depot.find_last_of('_');
            if (underscore_pos != string::npos) {
                instance_name = instance_depot.substr(0, underscore_pos);
                depot_position = instance_depot.substr(underscore_pos + 1);

                // 创建唯一键
                string key = instance_name + "_" + depot_position + "_" + to_string(drone_count);

                OptimalSolution opt;
                opt.instance_name = instance_name;
                opt.depot_position = depot_position;
                opt.drone_count = drone_count;
                opt.optimal_makespan = optimal_value;

                optimalMap[key] = opt;
                count++;

                cout << "Loaded optimal: " << key << " = " << optimal_value << endl;
            }
        }
    }

    file.close();
    cout << "Successfully loaded " << count << " optimal solutions." << endl;
    return optimalMap;
}

// 生成带时间戳的文件名
string generateTimestampFilename(const string& baseName, const string& extension) {
    auto now = chrono::system_clock::now();
    time_t now_time = chrono::system_clock::to_time_t(now);
    tm local_tm = *localtime(&now_time);

    stringstream ss;
    ss << baseName << "_"
        << put_time(&local_tm, "%Y%m%d_%H%M%S")
        << extension;
    return ss.str();
}

// ✅ 获取当天的日期文件夹路径（相对路径版本）
//    - 结果目录：./TSP-DS-result/YYYYMMDD/
//    - JSON目录：./TSP-DS-result-json/YYYYMMDD/
fs::path getDailyResultDirectory(bool isJson) {
    auto now = chrono::system_clock::now();
    time_t now_time = chrono::system_clock::to_time_t(now);
    tm local_tm = *localtime(&now_time);

    stringstream date_ss;
    date_ss << put_time(&local_tm, "%Y%m%d");
    string date_str = date_ss.str();

    fs::path base_dir = isJson ? fs::path("TSP-DS-result-json")
        : fs::path("TSP-DS-result");
    fs::path daily_dir = base_dir / date_str;

    fs::create_directories(daily_dir);
    return daily_dir;
}



static std::string joinInts(const std::vector<int>& v, const std::string& sep = "->") {
    std::string s;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) s += sep;
        s += std::to_string(v[i]);
    }
    return s;
}

static std::string formatDroneAssignments(const std::unordered_map<int, std::vector<int>>& mp) {
    // 稳定输出：按 drone_id 排序，避免 unordered_map 输出顺序每次不同
    std::vector<std::pair<int, std::vector<int>>> items;
    items.reserve(mp.size());
    for (const auto& kv : mp) items.push_back(kv);

    std::sort(items.begin(), items.end(),
        [](const std::pair<int, std::vector<int>>& a, const std::pair<int, std::vector<int>>& b) {
            return a.first < b.first;
        });

    // 格式：D0:{2 5 7}|D1:{3 9}
    std::string s;
    for (size_t i = 0; i < items.size(); ++i) {
        if (i) s += "|";
        int d = items[i].first;
        const auto& nodes = items[i].second;

        s += "D" + std::to_string(d) + ":{";
        for (size_t k = 0; k < nodes.size(); ++k) {
            if (k) s += " ";
            s += std::to_string(nodes[k]);
        }
        s += "}";
    }
    return s;
}

// ✅ JSON 单文件路径（相对目录拼接，跨平台）
static std::string makeJobJsonPath(const fs::path& daily_dir_json,
    const std::string& instance_name,
    int drone_count,
    int run_id)
{
    fs::path p = daily_dir_json /
        (instance_name + "_D" + std::to_string(drone_count) + "_run" + std::to_string(run_id) + ".json");
    return p.string();
}

void placeDepot(vector<pair<double, double>>& coords, const string& depot_position) {
    double min_x = 1e18, max_x = -1e18;
    double min_y = 1e18, max_y = -1e18;

    for (auto& p : coords) {
        min_x = min(min_x, p.first);
        max_x = max(max_x, p.first);
        min_y = min(min_y, p.second);
        max_y = max(max_y, p.second);
    }

    double depot_x, depot_y;

    if (depot_position == "C") {
        depot_x = (max_x + min_x) / 2.0;
        depot_y = (max_y + min_y) / 2.0;
    }
    else {  // "L"
        depot_x = min_x;
        depot_y = min_y;
    }

    coords.insert(coords.begin(), { depot_x, depot_y });
}

int findDroneStation(const vector<vector<double>>& drone_dist) {
    int N = (int)drone_dist.size();
    int best = 1;
    double best_val = 1e18;

    for (int i = 1; i < N; i++) {   // customers only
        double sum = 0.0;
        for (int j = 1; j < N; j++) {
            if (i == j) continue;
            sum += drone_dist[i][j];
        }
        double avg = sum / (N - 2);  // exclude depot & itself
        if (avg < best_val) {
            best_val = avg;
            best = i;
        }
    }
    return best;
}

// TSPLIB 常用的 nint：四舍五入到最近整数
static inline int nint(double x) {
    return static_cast<int>(std::floor(x + 0.5));
}

void setupDroneEligibility(TSPDSGraph& g, int K_percent, int depot)
{
    int N = (int)g.nodes.size();
    int customers = N - 1;  // exclude depot(0)

    double K = K_percent / 100.0;
    double L = 0.30;

    int target_drone = (int)std::ceil(customers * K);

    std::vector<bool> eligible(N, false);

    // 先全部默认 truck-only（后面把 eligible 的点改回来）
    g.is_truck_only.assign(N, true);
    g.is_truck_only[g.depot] = true;           // depot 永远 truck-only
    if (g.drone_station >= 0) g.is_truck_only[g.drone_station] = true; // station 永远 truck-only

    // -------- 1) weight-based non-eligible --------
    int denom = (int)std::floor(1.0 / ((1.0 - K) * L));
    if (denom < 1) denom = 1;

    std::vector<bool> is_weight_block(N, false);
    for (int n = 1; n < N; ++n) {  // customers only
        if (n == g.drone_station)
        {
            continue;
        }
        if (n % denom == 0) is_weight_block[n] = true;
    }

    // -------- 2) remaining customers sorted by distance to depot --------
    std::vector<std::pair<int, int>> dist_list;
    dist_list.reserve(N - 1);
    for (int i = 1; i < N; ++i) {
        double dx = g.nodes[i].first - g.nodes[depot].first;
        double dy = g.nodes[i].second - g.nodes[depot].second;
        int  d = nint(std::sqrt(dx * dx + dy * dy));   // 论文这里是“distance to depot”，通常用欧氏
        dist_list.push_back({ d, i });
    }
    std::sort(dist_list.begin(), dist_list.end());

    // 选出 eligible
    int count = 0;
    for (auto& pr : dist_list) {
        int v = pr.second;
        if (v == g.drone_station) continue;     // station 不作为可无人机服务客户
        if (is_weight_block[v]) continue;       // 重量原因不可无人机
        if (count >= target_drone) break;

        eligible[v] = true;
        g.is_truck_only[v] = false;             // ✅ 关键：eligible 的点不再是 truck-only
        ++count;
    }

    // station / depot 再强制一次（防止上面逻辑遗漏）
    eligible[g.depot] = false;
    if (g.drone_station >= 0) eligible[g.drone_station] = false;
    g.is_truck_only[g.depot] = true;
    if (g.drone_station >= 0) g.is_truck_only[g.drone_station] = true;

    g.is_drone_eligible = eligible;
}

// 随机选择 eligible（直到达到 K%），但通过 seed 保证可复现/跨进程一致
void setupDroneEligibilityRandom(TSPDSGraph& g, int K_percent, uint32_t seed)
{
    int N = (int)g.nodes.size();
    if (N <= 2) {
        g.is_drone_eligible.assign(N, false);
        g.is_truck_only.assign(N, true);
        return;
    }

    double K = K_percent / 100.0;

    // customers: 1..N-1，其中有一个是 station，不作为可无人机服务客户
    int customers_excl_station = N - 2; // 排除 depot(0) 与 station(1个)
    int target_drone = (int)std::ceil(customers_excl_station * K);

    // 默认全部 truck-only
    g.is_drone_eligible.assign(N, false);
    g.is_truck_only.assign(N, true);

    // depot / station 永远 truck-only & 不 eligible
    g.is_truck_only[g.depot] = true;
    if (g.drone_station >= 0) g.is_truck_only[g.drone_station] = true;
    g.is_drone_eligible[g.depot] = false;
    if (g.drone_station >= 0) g.is_drone_eligible[g.drone_station] = false;

    // -------- 2) 构造候选池：排除 depot / station / weight-block --------
    std::vector<int> pool;
    pool.reserve(N);

    for (int v = 1; v < N; ++v) {
        if (v == g.drone_station) continue;
        pool.push_back(v);
    }

    // -------- 3) 随机打乱并选前 target_drone 个 80%个 --------
    //随机数生成器用传入的 seed，保证跨进程一致，随机设置无人机 eligible 集合 但是各进程间一致
    std::mt19937 rng(seed);
    std::shuffle(pool.begin(), pool.end(), rng);

    int pick = std::min(target_drone, (int)pool.size());
    for (int i = 0; i < pick; ++i) {
        int v = pool[i];
        g.is_drone_eligible[v] = true;
        g.is_truck_only[v] = false;
    }

    // 再强制一次（保险）
    g.is_drone_eligible[g.depot] = false;
    if (g.drone_station >= 0) g.is_drone_eligible[g.drone_station] = false;
    g.is_truck_only[g.depot] = true;
    if (g.drone_station >= 0) g.is_truck_only[g.drone_station] = true;
}


TSPDSGraph parseTSPLIBFile(
    const string& filename,
    int drone_count,
    double speed_ratio,
    double drone_range,
    const string& depot_position)
{
    TSPDSGraph g;
    ifstream file(filename);

    int dimension = 0;
    vector<pair<double, double>> coords;
    string line;
    bool read_coord = false;

    while (getline(file, line)) {
        if (line.find("DIMENSION") != string::npos) {
            dimension = stoi(line.substr(line.find(":") + 1));
        }
        if (line.find("NODE_COORD_SECTION") != string::npos) {
            coords.resize(dimension);
            read_coord = true;
            continue;
        }
        if (read_coord) {
            if (line.find("EOF") != string::npos) break;
            istringstream iss(line);
            int id; double x, y;
            iss >> id >> x >> y;
            coords[id - 1] = { x,y };
        }
    }
    file.close();

    // 1. place depot according to paper
    placeDepot(coords, depot_position);

    // update dimension
    dimension = (int)coords.size();

    // init graph
    g.initialize(dimension);
    g.nodes = coords;

    // 2. build Manhattan truck / Euclidean drone
    g.initDistanceMatrices(coords, speed_ratio);

    // 3. drone station = min avg distance
    g.drone_station = findDroneStation(g.drone_time);

    // 4. drone eligible set (K=80%) —— 随机但全局一致
    {
        // 用 文件名 + depot_position + station 生成每个实例固定的 seed
        std::string key = fs::path(filename).filename().string() + "|" + depot_position + "|S" + std::to_string(g.drone_station);
        uint64_t h = fnv1a64(key) ^ ((uint64_t)g_elig_master_seed << 1);
        uint32_t seed = mix32(h);

        //setupDroneEligibilityRandom(g, 80, seed);

        setupDroneEligibility(g, 80, g.depot);
    }


    // 5. set remaining fields
    g.drone_count = drone_count;
    g.speed_ratio = speed_ratio;
    g.drone_range = drone_range;

    return g;
}

//-------------------------------------------------------------
// tau.csv / tauprime.csv: 方阵，读成二维 double 矩阵

static std::vector<std::vector<double>> readMatrixCsv(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open: " + path);

    std::vector<std::vector<double>> M;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> row;
        while (std::getline(ss, cell, ',')) {
            if (!cell.empty()) row.push_back(std::stod(cell));
        }
        if (!row.empty()) M.push_back(std::move(row));
    }
    return M;
}

// nodes.csv: 通常是 id,x,y,flag... 这里只取 id,x,y
static std::vector<std::pair<double, double>> readNodesXY(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open: " + path);

    std::vector<std::pair<double, double>> xy;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string a, b, c, d;
        if (!std::getline(ss, a, ',')) continue;
        std::getline(ss, b, ',');
        std::getline(ss, c, ',');
        int id = std::stoi(a);
        double x = std::stod(b);
        double y = std::stod(c);

        if ((int)xy.size() <= id) xy.resize(id + 1);
        xy[id] = { x,y };
    }
    return xy;
}

// Cprime.csv: 统一读成 int 列表
static std::vector<int> readCprime(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open: " + path);

    std::vector<int> C;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            if (!cell.empty()) C.push_back(std::stoi(cell));
        }
    }
    std::sort(C.begin(), C.end());
    C.erase(std::unique(C.begin(), C.end()), C.end());
    return C;
}

static bool isSquare(const std::vector<std::vector<double>>& M) {
    if (M.empty()) return false;
    int n = (int)M.size();
    for (auto& r : M) if ((int)r.size() != n) return false;
    return true;
}

// 处理 Murray 常见的 “start depot=0, end depot=c+1” 情况：默认删掉最后一个点
static void dropLastNodeAndMatrix(
    std::vector<std::pair<double, double>>& nodes,
    std::vector<std::vector<double>>& tau,
    std::vector<std::vector<double>>& taup
) {
    int n = (int)nodes.size();
    if (n <= 2) return;
    nodes.pop_back();
    tau.pop_back();
    taup.pop_back();
    for (auto& r : tau)  r.pop_back();
    for (auto& r : taup) r.pop_back();
}

// ====== 核心：Murray 数据 -> 你的 TSPDSGraph ======
TSPDSGraph parseMurrayInstance(
    const std::string& folder,
    int drone_count,
    double drone_range_minutes   // 论文用 30.0
) {
    TSPDSGraph g;

    // 1) 读四个文件
    auto nodes = readNodesXY(folder + "/nodes.csv");
    auto tau = readMatrixCsv(folder + "/tau.csv");
    auto taup = readMatrixCsv(folder + "/tauprime.csv");
    auto Cprime = readCprime(folder + "/Cprime.csv");

    if (!isSquare(tau) || !isSquare(taup))
        throw std::runtime_error("tau/tauprime not square matrices.");

    if ((int)nodes.size() != (int)tau.size() || (int)nodes.size() != (int)taup.size())
        throw std::runtime_error("nodes size != tau/tauprime dimension.");

    // 2) 可选：删掉 end depot
    auto nearEq = [&](double a, double b) { return std::fabs(a - b) < 1e-9; };
    bool lastLooksLikeEndDepot =
        nearEq(nodes.front().first, nodes.back().first) &&
        nearEq(nodes.front().second, nodes.back().second);

    if (lastLooksLikeEndDepot) {
        dropLastNodeAndMatrix(nodes, tau, taup);
    }

    int N = (int)nodes.size(); // 现在是 0..c , 0是depot
    g.initialize(N);
    g.nodes = nodes;
    g.depot = 0;
    g.drone_count = drone_count;
    g.drone_range = drone_range_minutes;
    g.speed_ratio = 1;

    // 3) 直接用矩阵作为时间
    g.truck_time = tau;
    g.drone_time = taup;

    // 4) station = 覆盖最多客户的客户点（往返<=range）
    std::unordered_set<int> Cset(Cprime.begin(), Cprime.end());

    auto roundTrip = [&](int s, int j)->double {
        return g.drone_time[s][j] + g.drone_time[j][s];
        };

    int bestStation = 1;
    int bestCover = -1;

    for (int s = 1; s < N; ++s) {
        int cover = 0;
        for (int j : Cprime) {
            if (j <= 0 || j >= N) continue;
            if (j == s) continue;
            if (roundTrip(s, j) <= drone_range_minutes + 1e-9) cover++;
        }
        if (cover > bestCover) {
            bestCover = cover;
            bestStation = s;
        }
    }
    g.drone_station = bestStation;

    // 5) is_drone_station
    g.is_drone_station.assign(N, false);
    if (g.drone_station >= 0) g.is_drone_station[g.drone_station] = true;

    // 6) eligible/truck-only：Cprime + 航程 + 排除 depot/station
    g.is_drone_eligible.assign(N, false);
    g.is_truck_only.assign(N, true);

    g.is_truck_only[g.depot] = true;
    if (g.drone_station >= 0) g.is_truck_only[g.drone_station] = true;

    for (int v = 1; v < N; ++v) {
        if (v == g.drone_station) continue;

        bool eligibleByWeight = (Cset.count(v) > 0);
        bool eligibleByRange = (roundTrip(g.drone_station, v) <= drone_range_minutes + 1e-9);

        bool ok = eligibleByWeight && eligibleByRange;
        g.is_drone_eligible[v] = ok;
        g.is_truck_only[v] = !ok;
    }

    g.is_drone_eligible[g.depot] = false;
    if (g.drone_station >= 0) g.is_drone_eligible[g.drone_station] = false;

    return g;
}

static std::vector<std::string> listSubDirs(const std::string& root) {
    std::vector<std::string> dirs;
    for (const auto& entry : fs::directory_iterator(root)) {
        if (entry.is_directory()) dirs.push_back(entry.path().string());
    }
    return dirs;
}

static void sampleK(std::vector<std::string>& v, int K, std::mt19937& rng) {
    std::shuffle(v.begin(), v.end(), rng);
    if ((int)v.size() > K) v.resize(K);
}


static bool lookupMurrayOptimal(const std::map<std::string, OptimalSolution>& optimalSolutions,
    const std::string& instance_base,
    int drone_count,
    double& out_opt)
{
    out_opt = -1.0;
    std::vector<std::string> keys;
    keys.push_back(instance_base + "_MURRAY_H_" + std::to_string(drone_count));
    keys.push_back(instance_base + "_MURRAY_" + std::to_string(drone_count));
    keys.push_back(instance_base + "_H_" + std::to_string(drone_count));
    keys.push_back(instance_base + "_C_" + std::to_string(drone_count));

    for (const auto& k : keys) {
        auto it = optimalSolutions.find(k);
        if (it != optimalSolutions.end()) {
            out_opt = it->second.optimal_makespan;
            return true;
        }
    }
    return false;
}

// 处理单个 Murray 文件夹：folder 内需要包含 nodes.csv / tau.csv / tauprime.csv / Cprime.csv
void processMurrayFolder(const std::string& folderPath,
    const fs::path& daily_dir_json,
    int drone_count,
    double drone_range_minutes,
    int run_id,
    std::ofstream& result_file,
    const std::map<std::string, OptimalSolution>& optimalSolutions,
    std::mutex& result_mutex)
{
    auto start_time = chrono::high_resolution_clock::now();
    std::string instance_base = fs::path(folderPath).filename().string();

    try {
        TSPDSGraph graph = parseMurrayInstance(folderPath, drone_count, drone_range_minutes);

        cout << "\n==========================================" << endl;
        cout << "Processing Murray FOLDER: " << folderPath
            << " (drones: " << drone_count << ")" << endl;
        cout << "==========================================" << endl;
        cout << "Problem instance details:" << endl;
        cout << "Number of nodes: " << graph.nodes.size() << endl;
        cout << "Depot node: " << graph.depot
            << " Depot x: " << graph.nodes[graph.depot].first
            << " Depot y: " << graph.nodes[graph.depot].second << endl;
        cout << "Drone station node: " << graph.drone_station
            << " Drone x: " << graph.nodes[graph.drone_station].first
            << " Drone y: " << graph.nodes[graph.drone_station].second << endl;
        cout << "Number of drones: " << graph.drone_count << endl;

        TSPDSSolver solver(graph, run_id);

        std::string seed_key = instance_base + "|MURRAY|D" + std::to_string(drone_count)
            + "|run" + std::to_string(run_id);
        std::size_t h = std::hash<std::string>{}(seed_key);
        auto now = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        h ^= static_cast<std::size_t>(now) + (static_cast<std::size_t>(now) << 7);
        unsigned int seed = static_cast<unsigned int>(h & 0xFFFFFFFFu);

        solver.setRandomSeed(seed);
        solver.enableLogging(true);
        solver.setLogFilename(makeJobJsonPath(daily_dir_json, instance_base, drone_count, run_id));

        double optimal_makespan = -1.0;
        double early_stop = -1.0;
        if (lookupMurrayOptimal(optimalSolutions, instance_base, drone_count, optimal_makespan)) {
            early_stop = optimal_makespan;
        }

        // -------- CPLEX F2 求解（Murray，与第一份代码保持一致）--------
        TSPDSSolution sol_f2;
        bool f2_ok = false;
        double f2_stop = early_stop;
        {
            CplexF2Solver::Params p;
            p.time_limit_sec = 300;
            p.threads = 8;
            p.mip_gap = 0.0;
            p.verbose = true;

            CplexF2Solver solver_cplex(p);
            f2_ok = solver_cplex.solve(graph, sol_f2);

            if (f2_ok) {
                cout << "[CPLEX F2] T=" << sol_f2.makespan
                    << "  Tt=" << sol_f2.truck_completion_time
                    << "  Ta=" << sol_f2.station_activation_time
                    << "  Td=" << sol_f2.drone_completion_time << endl;

                if (sol_f2.makespan > 0 && std::isfinite(sol_f2.makespan)) {
                    f2_stop = sol_f2.makespan;
                }
                else if (sol_f2.cplex_best_integer > 0 && std::isfinite(sol_f2.cplex_best_integer)) {
                    f2_stop = sol_f2.cplex_best_integer;
                }

                double f2_gap_percent = -1.0;
                if (optimal_makespan > 0) {
                    f2_gap_percent = ((sol_f2.makespan - optimal_makespan) / optimal_makespan) * 100.0;
                }

                auto f2_end_time = chrono::high_resolution_clock::now();
                auto f2_duration = chrono::duration_cast<chrono::milliseconds>(f2_end_time - start_time);

                std::string truckStrF2 = joinInts(sol_f2.truck_route, "->");
                std::string droneStrF2 = formatDroneAssignments(sol_f2.drone_assignments);

                {
                    std::lock_guard<std::mutex> lock(result_mutex);
                    result_file << instance_base << ","
                        << "MURRAY_F2" << ","
                        << drone_count << ","
                        << run_id << ","
                        << seed << ","
                        << graph.nodes.size() << ","
                        << sol_f2.makespan << ","
                        << f2_duration.count() / 1000.0 << ","
                        << optimal_makespan << ","
                        << f2_gap_percent << ","
                        << sol_f2.max_resAt << ","
                        << sol_f2.total_iter << ","
                        << sol_f2.truck_completion_time << ","
                        << sol_f2.drone_completion_time << ","
                        << sol_f2.station_activation_time << ","
                        << sol_f2.dorne_visit_dep << ","
                        << sol_f2.cplex_best_integer << ","
                        << sol_f2.cplex_best_bound << ","
                        << sol_f2.cplex_mip_gap << ","
                        << sol_f2.cplex_find_best_time << ","
                        << sol_f2.cplex_feasible_solution << ","
                        << csvEscape(sol_f2.cplex_feasible_info) << ","
                        << csvEscape(truckStrF2) << ","
                        << csvEscape(droneStrF2) << ","
                        << sol_f2.init_Id
                        << "\n";
                }
            }
            else {
                cout << "[CPLEX F2] solve failed." << endl;
            }
        }

        TSPDSSolution solution = solver.solve(f2_stop);

        auto end_time = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

        double gap_percent = -1.0;
        if (optimal_makespan > 0) {
            gap_percent = ((solution.makespan - optimal_makespan) / optimal_makespan) * 100.0;
        }

        cout << "\n=== SOLUTION RESULTS ===" << endl;
        cout << "Best makespan (Heuristic): " << solution.makespan << endl;
        cout << "Computation time: " << duration.count() / 1000.0 << " s" << endl;
        if (optimal_makespan > 0) {
            cout << "Optimal makespan: " << optimal_makespan << endl;
            cout << "Gap from optimal: " << gap_percent << "%" << endl;
        }

        std::string truckStr = joinInts(solution.truck_route, "->");
        std::string droneStr = formatDroneAssignments(solution.drone_assignments);

        {
            std::lock_guard<std::mutex> lock(result_mutex);
            result_file << instance_base << ","
                << "MURRAY_H" << ","
                << drone_count << ","
                << run_id << ","
                << seed << ","
                << graph.nodes.size() << ","
                << solution.makespan << ","
                << solution.find_best_time << ","
                << optimal_makespan << ","
                << gap_percent << ","
                << solution.max_resAt << ","
                << solution.total_iter << ","
                << solution.truck_completion_time << ","
                << solution.drone_completion_time << ","
                << solution.station_activation_time << ","
                << solution.dorne_visit_dep << ","
                << solution.cplex_best_integer << ","
                << solution.cplex_best_bound << ","
                << solution.cplex_mip_gap << ","
                << solution.cplex_find_best_time << ","
                << solution.cplex_feasible_solution << ","
                << csvEscape(solution.cplex_feasible_info) << ","
                << csvEscape(truckStr) << ","
                << csvEscape(droneStr) << ","
                << solution.init_Id
                << "\n";
        }
    }
    catch (const exception& e) {
        cerr << "Error processing Murray folder " << folderPath << ": " << e.what() << endl;
        std::lock_guard<std::mutex> lock(result_mutex);
        result_file << instance_base << ","
            << "MURRAY_H" << ","
            << drone_count << ","
            << run_id << ","
            << "ERROR," << csvEscape(e.what()) << "\n";
    }
}

// 处理单个TSPLIB文件
void processTSPFile(const string& filepath,
    const string& result_filepath_json,
    const string& depot_position,
    int drone_count,
    int run_id,
    ofstream& result_file,
    const map<string, OptimalSolution>& optimalSolutions,
    std::mutex& result_mutex,
    const std::unordered_map<std::string, ExactResult>& exactMap)
{
    auto start_time = chrono::high_resolution_clock::now();

    try {
        std::string instance_name, depot_pos;
        int K = 80;
        parseCsvName(fs::path(filepath), instance_name, depot_pos, K);

        //＜280数据以及设定
        //TSPDSGraph graph = parseOriginCsvFile(filepath, drone_count, 0.5, std::numeric_limits<double>::max());

        //>280个点的 数据以及解析图
        TSPDSGraph graph = parseTSPLIBFile(filepath, drone_count, 0.5, std::numeric_limits<double>::max(), depot_position);



        cout << "\n==========================================" << endl;
        cout << "Processing TSPLIB FILE: " << filepath << " (depot: " << depot_pos << ", drones: " << drone_count << ")" << endl;
        cout << "==========================================" << endl;

        cout << "Problem instance details:" << endl;
        cout << "Number of nodes: " << graph.nodes.size() << endl;
        cout << "Depot node: " << graph.depot
            << " Depot x: " << graph.nodes[graph.depot].first
            << " Depot y: " << graph.nodes[graph.depot].second << endl;
        cout << "Drone station node: " << graph.drone_station
            << " Drone x: " << graph.nodes[graph.drone_station].first
            << " Drone y: " << graph.nodes[graph.drone_station].second << endl;
        cout << "Number of drones: " << graph.drone_count << endl;

        // -------- 启发式求解 --------
        TSPDSSolver solver(graph, run_id);

        std::string instance_base = fs::path(filepath).stem().string();
        std::size_t h1 = std::hash<std::string>{}(instance_base);
        std::size_t h2 = std::hash<std::string>{}(depot_position);
        std::size_t h3 = static_cast<std::size_t>(drone_count);
        std::size_t h4 = static_cast<std::size_t>(run_id);

        std::size_t combined = h1 ^ (h2 << 1) ^ (h3 << 16) ^ (h4 << 24);
        auto now = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        std::size_t time_part = static_cast<std::size_t>(now);
        combined ^= (time_part + (time_part << 7));

        unsigned int seed = static_cast<unsigned int>(combined & 0xFFFFFFFFu);

        solver.setRandomSeed(seed);
        solver.enableLogging(true);
        solver.setLogFilename(result_filepath_json);

        double early_stop = -1;
        string search_key = instance_name + "_" + depot_pos + "_" + to_string(drone_count);

        auto it = optimalSolutions.find(search_key);
        if (it != optimalSolutions.end()) {
            early_stop = it->second.optimal_makespan;
        }

        TSPDSSolution solution;
        solution = solver.solve(early_stop);



        // -------- 统计耗时 & 输出 --------
        auto end_time = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

        cout << "\n=== SOLUTION RESULTS ===" << endl;
        cout << "Best makespan (Heuristic): " << solution.makespan << endl;
        cout << "Computation time: " << duration.count() / 1000.0 << " s" << endl;

        // -------- 查最优对比 --------

        std::string k = exactKey(filepath, depot_position, drone_count);
        ExactResult ex;
        auto itex = exactMap.find(k);
        if (itex != exactMap.end()) ex = itex->second;

        //// 用 CPLEX 的 makespan 当 optimal_makespan（若 ok）

        // -------- 查最优对比 --------
        string search_key2 = instance_name + "_" + depot_pos + "_" + to_string(drone_count);

        double optimal_makespan = -1;
        double gap_percent = -1;

        auto it2 = optimalSolutions.find(search_key2);
        if (it2 != optimalSolutions.end()) {
            optimal_makespan = it2->second.optimal_makespan;
            gap_percent = ((solution.makespan - optimal_makespan) / optimal_makespan) * 100.0;
            cout << "Optimal makespan: " << optimal_makespan << endl;
            cout << "Gap from optimal: " << gap_percent << "%" << endl;
        }
        else {
            cout << "No optimal solution found in database for this configuration." << endl;
        }

        // 同步 cplex 信息到 solution
        solution.cplex_status = ex.cplex_status;
        solution.cplex_status_name = ex.cplex_status_name;
        solution.cplex_result_type = ex.cplex_result_type;
        solution.cplex_best_integer = ex.best_integer;
        solution.cplex_best_bound = ex.best_bound;
        solution.cplex_mip_gap = ex.mip_gap;
        solution.cplex_find_best_time = ex.find_best_time_sec;
        solution.cplex_feasible_solution = ex.feasible_solution;
        solution.cplex_feasible_info = ex.feasible_info;


        // -------- 写 CSV（必须锁）--------
        std::string truckStr = joinInts(solution.truck_route, "->");
        std::string droneStr = formatDroneAssignments(solution.drone_assignments);

        {
            std::lock_guard<std::mutex> lock(result_mutex);

            result_file << fs::path(filepath).filename().string() << ","
                << depot_position << ","
                << drone_count << ","
                << run_id << ","
                << seed << ","
                << graph.nodes.size() << ","
                << solution.makespan << ","
                << solution.find_best_time << ","
                << optimal_makespan << ","
                << gap_percent << ","
                << solution.max_resAt << ","
                << solution.total_iter << ","
                << solution.truck_completion_time << ","
                << solution.drone_completion_time << ","
                << solution.station_activation_time << ","
                << solution.dorne_visit_dep << ","
                << solution.cplex_status << ","
                << csvEscape(solution.cplex_status_name) << ","
                << csvEscape(solution.cplex_result_type) << ","
                << solution.cplex_best_integer << ","
                << solution.cplex_best_bound << ","
                << solution.cplex_mip_gap << ","
                << solution.cplex_find_best_time << ","
                << solution.cplex_feasible_solution << ","
                << csvEscape(solution.cplex_feasible_info) << ","
                << csvEscape(truckStr) << ","
                << csvEscape(droneStr) << ","
                << solution.init_Id
                << "\n";
        }
    }
    catch (const exception& e) {
        cerr << "Error processing file " << filepath << ": " << e.what() << endl;

        string instance_base = fs::path(filepath).stem().string();
        string search_key = instance_base + "_" + depot_position + "_" + to_string(drone_count);

        std::lock_guard<std::mutex> lock(result_mutex);

        result_file << fs::path(filepath).filename().string() << ","
            << depot_position << ","
            << drone_count << ","
            << run_id << ","
            << "ERROR," << e.what() << ",";

        auto it = optimalSolutions.find(search_key);
        if (it != optimalSolutions.end()) {
            result_file << it->second.optimal_makespan << ",N/A\n";
        }
        else {
            result_file << "N/A,N/A\n";
        }
    }
}

int main() {
    std::mutex result_mutex;      // 保护 CSV 写入
    const int NUM_REPEATS = 1;     // 并行重复次数
    // ========== 进程数==========
    int WORKERS = 1;
    if (WORKERS < 1) WORKERS = 1;

    // 定义无人机数量数组
    vector<int> drone_counts = { 1,3,5 }; // 1,3,5,7,9
    vector<string> depot_positions = { "L" }; // "1" "C", "L"
    bool isSmallInstance = false; // Murray小实例=true；TSPLIB=false
    bool isRunF2Cplex = false; // 是否运行 F2 求解 Exact
    bool isRunBenders = true; // 是否运行 Benders 求解 Exact

    // ✅ 相对路径：以“程序工作目录”为基准  
    fs::path input_dir = fs::path("data") / "tsp_origan";
    // Murray 小实例根目录：每个子目录包含 nodes.csv / tau.csv / tauprime.csv / Cprime.csv
    fs::path murray_root = fs::path("data") / "Murray_Chu_2015_test_data" / "PDSTSP" / "temp";
    double murray_drone_range_minutes = 30.0;
    int murray_sample_limit = 20; // 0 表示全部；需要抽样时改成 20 等数量
    fs::path optimal_file = fs::path("data") / "TSP-DS-OPT.txt";


    // 读取最优解文件（相对）
    auto optimalSolutions = readOptimalSolutions(optimal_file.string());

    // 获取当天结果目录（相对）
    fs::path daily_dir = getDailyResultDirectory(false);
    cout << "Daily result directory: " << daily_dir << endl;

    fs::path daily_dir_json = getDailyResultDirectory(true);
    cout << "Daily json directory: " << daily_dir_json << endl;



    // 生成带时间戳的结果文件名并保存到当天目录（相对）
    string result_filename = generateTimestampFilename("tsp_ds_results", ".csv");
    fs::path result_filepath = daily_dir / result_filename;

    string result_filename_json = generateTimestampFilename("tsp_ds_results", ".json");
    fs::path result_filepath_json = daily_dir_json / result_filename_json;



    // 重要：父进程不用开多线程；每个子进程也固定 1 线程，保证每个进程≈100%
    omp_set_dynamic(0);

    // ========== 先为每个 worker 准备独立 CSV 路径 ==========
    std::vector<fs::path> worker_csv_paths;
    worker_csv_paths.reserve(WORKERS);


    for (int w = 0; w < WORKERS; ++w) {
        fs::path p = daily_dir / (std::string("worker_") + std::to_string(w) + "_" + result_filename);
        worker_csv_paths.push_back(p);
    }

    int file_count = 0;

    // 1) 根据实例类型收集任务源
    std::vector<std::string> tsp_files;
    std::vector<std::string> murray_folders;

    if (!isSmallInstance) {
        for (const auto& entry : fs::directory_iterator(input_dir)) {
            if (!entry.is_regular_file()) continue;
            std::string ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(),
                [](unsigned char c) { return (char)std::tolower(c); });
            if (ext == ".csv" || ext == ".tsp") {
                tsp_files.push_back(entry.path().string());
            }
        }
        std::sort(tsp_files.begin(), tsp_files.end());
        file_count = (int)tsp_files.size();
    }
    else {
        if (!fs::exists(murray_root)) {
            fs::path alt = fs::path("data") / "Murray_Chu_2015_test_data" / "PDSTSP" / "PDSTSP_20_customer_problems";
            if (fs::exists(alt)) murray_root = alt;
        }
        murray_folders = listSubDirs(murray_root.string());
        std::sort(murray_folders.begin(), murray_folders.end());
        if (murray_sample_limit > 0 && (int)murray_folders.size() > murray_sample_limit) {
            std::mt19937 rng((unsigned)std::chrono::high_resolution_clock::now().time_since_epoch().count());
            sampleK(murray_folders, murray_sample_limit, rng);
            std::sort(murray_folders.begin(), murray_folders.end());
        }
        file_count = (int)murray_folders.size();
    }

    // 2) 构造 jobs
    struct Job {
        std::string path;
        std::string depot;
        int drone_count;
        int run_id;
        bool is_murray;
    };

    std::vector<Job> jobs;
    jobs.reserve((size_t)std::max(1, file_count) * depot_positions.size() * drone_counts.size() * NUM_REPEATS);

    for (int run_id = 0; run_id < NUM_REPEATS; ++run_id) {
        if (!isSmallInstance) {
            for (const std::string& filename : tsp_files) {
                for (const std::string& depot_position : depot_positions) {
                    for (int dc : drone_counts) {
                        jobs.push_back(Job{ filename, depot_position, dc, run_id, false });
                    }
                }
            }
        }
        else {
            for (const std::string& folder : murray_folders) {
                for (int dc : drone_counts) {
                    jobs.push_back(Job{ folder, "MURRAY_H", dc, run_id, true });
                }
            }
        }
    }

    // ========== 预计算：每个 (instance, depot, drone_count) 的 CPLEX 只跑一次 ==========
    std::unordered_map<std::string, ExactResult> exactMap;
    exactMap.reserve(tsp_files.size() * depot_positions.size() * drone_counts.size());

    if (!isSmallInstance && isRunF2Cplex) {
        for (const std::string& filename : tsp_files) {
            for (const std::string& depot_position : depot_positions) {
                for (int dc : drone_counts) {

                    std::string k = exactKey(filename, depot_position, dc);
                    if (exactMap.find(k) != exactMap.end()) continue;

                    ExactResult ex;
                    try {
                        TSPDSGraph graph = parseTSPLIBFile(filename, dc, 0.5,
                            std::numeric_limits<double>::max(), depot_position);

                        CplexF2Solver::Params p;
                        p.time_limit_sec = 3600;
                        p.threads = 8;
                        p.mip_gap = 0.0;
                        p.verbose = true;

                        CplexF2Solver solver_cplex(p);
                        TSPDSSolution sol_cplex;
                        bool ok = solver_cplex.solve(graph, sol_cplex);

                        ex.ok = ok;
                        if (ok) {
                            ex.makespan = sol_cplex.makespan;
                            ex.Tt = sol_cplex.truck_completion_time;
                            ex.Ta = sol_cplex.station_activation_time;
                            ex.Td = sol_cplex.drone_completion_time;
                            ex.best_integer = sol_cplex.cplex_best_integer;
                            ex.best_bound = sol_cplex.cplex_best_bound;
                            ex.mip_gap = sol_cplex.cplex_mip_gap;

                            std::cout << "[CPLEX once] " << k
                                << " T=" << ex.makespan
                                << " Tt=" << ex.Tt
                                << " Ta=" << ex.Ta
                                << " Td=" << ex.Td
                                << "\n";
                        }
                        else {
                            std::cout << "[CPLEX once] " << k << " solve failed.\n";
                        }
                    }
                    catch (const std::exception& e) {
                        ex.ok = false;
                        std::cerr << "[CPLEX once] " << k << " exception: " << e.what() << "\n";
                    }

                    exactMap.emplace(k, ex);
                }
            }
        }
    }

    if (!isSmallInstance && isRunBenders) {
        for (const std::string& filename : tsp_files) {
            for (const std::string& depot_position : depot_positions) {
                for (int dc : drone_counts) {

                    std::string k = exactKey(filename, depot_position, dc);
                    if (exactMap.find(k) != exactMap.end()) continue;

                    ExactResult ex;
                    try {

                        //TSPDSGraph graph = parseOriginCsvFile(filename, dc, 0.5, std::numeric_limits<double>::max());

                        //>280个点的 数据以及解析图
                        TSPDSGraph graph = parseTSPLIBFile(filename, dc, 0.5, std::numeric_limits<double>::max(), depot_position);

                        CplexBendersSolver::Params p;
                        p.time_limit_sec = 7200;
                        p.threads = 16;
                        p.mip_gap = 0.0;
                        p.verbose = true;

                        CplexBendersSolver solver_bendercplex(p);
                        TSPDSSolution sol_cplex;
                        bool ok = solver_bendercplex.solve(graph, sol_cplex);

                        ex.ok = ok;
                        if (ok) {
                            ex.ok = ok;
                            ex.makespan = sol_cplex.makespan;
                            ex.Tt = sol_cplex.truck_completion_time;
                            ex.Ta = sol_cplex.station_activation_time;
                            ex.Td = sol_cplex.drone_completion_time;

                            ex.best_integer = sol_cplex.cplex_best_integer;
                            ex.best_bound = sol_cplex.cplex_best_bound;
                            ex.mip_gap = sol_cplex.cplex_mip_gap;
                            ex.find_best_time_sec = sol_cplex.cplex_find_best_time;

                            ex.feasible_solution = sol_cplex.cplex_feasible_solution;
                            ex.feasible_info = sol_cplex.cplex_feasible_info;

                            ex.cplex_status = sol_cplex.cplex_status;
                            ex.cplex_status_name = sol_cplex.cplex_status_name;
                            ex.cplex_result_type = sol_cplex.cplex_result_type;

                            std::cout << "[CPLEX once] " << k
                                << " T=" << ex.makespan
                                << " Tt=" << ex.Tt
                                << " Ta=" << ex.Ta
                                << " Td=" << ex.Td
                                << "\n";
                        }
                        else {
                            ex.ok = ok;
                            ex.makespan = sol_cplex.makespan;
                            ex.Tt = sol_cplex.truck_completion_time;
                            ex.Ta = sol_cplex.station_activation_time;
                            ex.Td = sol_cplex.drone_completion_time;

                            ex.best_integer = sol_cplex.cplex_best_integer;
                            ex.best_bound = sol_cplex.cplex_best_bound;
                            ex.mip_gap = sol_cplex.cplex_mip_gap;
                            ex.find_best_time_sec = sol_cplex.cplex_find_best_time;

                            ex.feasible_solution = sol_cplex.cplex_feasible_solution;
                            ex.feasible_info = sol_cplex.cplex_feasible_info;

                            ex.cplex_status = sol_cplex.cplex_status;
                            ex.cplex_status_name = sol_cplex.cplex_status_name;
                            ex.cplex_result_type = sol_cplex.cplex_result_type;
                            std::cout << "[CPLEX once] " << k << " solve failed.\n";
                        }
                    }
                    catch (const std::exception& e) {
                        ex.ok = false;
                        std::cerr << "[CPLEX once] " << k << " exception: " << e.what() << "\n";
                    }

                    exactMap.emplace(k, ex);
                }
            }
        }
    }



    std::cout << "jobs=" << jobs.size() << "\n";

    // ========== fork 出 WORKERS 个子进程 ==========
    std::vector<pid_t> pids;
    pids.reserve(WORKERS);

    for (int w = 0; w < WORKERS; ++w) {
        pid_t pid = fork();
        if (pid == 0) {
            // ---------- 子进程 ----------
            omp_set_num_threads(1); // ✅ 每个进程只跑 1 线程 -> top 显示 100%

            std::ofstream wf(worker_csv_paths[w].string());
            if (!wf.is_open()) {
                std::cerr << "Worker " << w << " cannot open " << worker_csv_paths[w] << "\n";
                _exit(2);
            }

            // 写表头（每个 worker 文件都有表头，后面合并时跳过）
            wf << "Instance,DepotPosition,DroneCount,RunId,Seed,Nodes,"
                "Makespan,Time,OptimalMakespan,GapPercent,"
                "Find_Max_ResIter,Total_Iter,truck_complete_time,drone_complete_time,drone_active_time,Drone_dopsition,"
                "cplex_status,cplex_status_name,cplex_result_type,"
                "cplex_best_integer,cplex_best_bound,cplex_mip_gap,cplex_find_best_time,"
                "cplex_feasible_solution,cplex_feasible_info,"
                "TruckRoute,DroneTasks,Init_id\n";

            std::mutex dummy_mutex; // 进程内锁即可（其实单文件写不冲突）

            // 每个 worker 处理自己负责的 jobs：i % WORKERS == w
            for (int i = 0; i < (int)jobs.size(); ++i) {
                if (i % WORKERS != w) continue;

                const auto& job = jobs[i];
                std::string instance_base = job.is_murray
                    ? fs::path(job.path).filename().string()
                    : fs::path(job.path).stem().string();
                std::string job_json = makeJobJsonPath(daily_dir_json, instance_base, job.drone_count, job.run_id);

                if (job.is_murray) {
                    processMurrayFolder(job.path,
                        daily_dir_json,
                        job.drone_count,
                        murray_drone_range_minutes,
                        job.run_id,
                        wf,
                        optimalSolutions,
                        dummy_mutex);
                }
                else {
                    processTSPFile(job.path,
                        job_json,
                        job.depot,
                        job.drone_count,
                        job.run_id,
                        wf,
                        optimalSolutions,
                        dummy_mutex,
                        exactMap);
                }
            }

            wf.close();
            _exit(0);
        }

        if (pid > 0) {
            // 父进程记录子 pid
            pids.push_back(pid);
        }
        else {
            perror("fork");
            // fork 失败：停止继续开进程
            break;
        }
    }

    // ========== 父进程等待所有 worker ==========
    for (pid_t pid : pids) {
        int status = 0;
        waitpid(pid, &status, 0);
    }

    // ========== 合并 worker CSV -> 总 CSV ==========

    std::ofstream result_file(result_filepath.string());
    if (!result_file.is_open()) {
        std::cerr << "Error: Could not create final result file " << result_filepath << "\n";
        return 1;
    }

    const std::string header =
        "Instance,DepotPosition,DroneCount,RunId,Seed,Nodes,"
        "Makespan,Time,OptimalMakespan,GapPercent,"
        "Find_Max_ResIter,Total_Iter,truck_complete_time,drone_complete_time,drone_active_time,Drone_dopsition,"
        "cplex_status,cplex_status_name,cplex_result_type,"
        "cplex_best_integer,cplex_best_bound,cplex_mip_gap,cplex_find_best_time,"
        "cplex_feasible_solution,cplex_feasible_info,"
        "TruckRoute,DroneTasks,Init_id";

    result_file << header << "\n";

    // --- 汇总：按 (Instance, DroneCount) 取 makespan 最小的一行 ---
    struct BestRow {
        double makespan = std::numeric_limits<double>::infinity();
        std::vector<std::string> cells; // 原始 cells（未 escape 的）
    };

    std::unordered_map<std::string, BestRow> bestMap;
    bestMap.reserve(1024);

    auto makeKey = [](const std::string& instance, const std::string& droneCountStr) {
        return instance + "|D=" + droneCountStr;
        };

    // 逐个 worker 文件追加（跳过第一行表头）
    for (int w = 0; w < (int)worker_csv_paths.size(); ++w) {
        std::ifstream in(worker_csv_paths[w].string());
        if (!in.is_open()) continue;

        std::string line;
        bool first = true;
        while (std::getline(in, line)) {
            if (first) { first = false; continue; } // skip worker header
            if (line.empty()) continue;

            // 先把明细直接写入总表
            result_file << line << "\n";

            // 再解析做 best 统计
            auto cells = splitCsvRow(line);
            if (cells.size() < 25) continue; // 列数不够就跳过（避免异常行）

            const std::string& instance = cells[0];
            const std::string& droneCountStr = cells[2];

            double mk = 0.0;
            try {
                mk = std::stod(cells[6]); // Makespan 第7列（从0开始）
            }
            catch (...) {
                continue;
            }

            std::string k = makeKey(instance, droneCountStr);
            auto& br = bestMap[k];
            if (mk < br.makespan) {
                br.makespan = mk;
                br.cells = std::move(cells);
            }
        }
    }

    // --- 在表格末尾追加“汇总区” ---
    {
        std::vector<std::string> mark(25, "");
        mark[0] = "SUMMARY";
        mark[1] = "BEST_PER_(Instance,DroneCount)";
        result_file << joinCsvRow(mark) << "\n";
    }

    // 为了输出稳定排序一下（按 Instance，再按 DroneCount）
    struct Item { std::string instance; int dc; BestRow br; };
    std::vector<Item> items;
    items.reserve(bestMap.size());

    for (auto& kv : bestMap) {
        if (kv.second.cells.empty()) continue;
        auto& c = kv.second.cells;
        int dc = 0;
        try { dc = std::stoi(c[2]); }
        catch (...) { dc = 0; }
        items.push_back(Item{ c[0], dc, kv.second });
    }

    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
        if (a.instance != b.instance) return a.instance < b.instance;
        return a.dc < b.dc;
        });

    // 输出每个组合的 best 行：把 DepotPosition 改成 "BEST" 方便识别
    for (auto& it : items) {
        auto cells = it.br.cells;
        if (cells.size() < 25) continue;
        cells[1] = "BEST";  // DepotPosition 列，用来标记这是汇总行
        result_file << joinCsvRow(cells) << "\n";
    }

    result_file.close();

    // 合并完成后：删除 worker 临时文件
    for (const auto& p : worker_csv_paths) {
        std::error_code ec;
        fs::remove(p, ec);
        if (ec) {
            std::cerr << "Warning: cannot remove " << p << " : " << ec.message() << "\n";
        }
    }

    result_file.close();

    return 0;
}
