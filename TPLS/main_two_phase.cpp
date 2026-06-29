// ======================= main.cpp (Multi-process workers + SUMMARY stats) =======================
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>
#include <cmath>
#include <stdexcept>
#include <random>
#include <cctype>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <cstdlib>
#include <map>

#include <omp.h>

#ifndef _WIN32
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#else
#error "This multi-process worker version uses fork() and only supports Linux/Unix. Use OpenMP threads on Windows."
#endif

#include "MTSPDSGraph.h"
#include "TwoPhaseMatheuristic.h"

namespace fs = std::filesystem;
using namespace std;

// 数据结构存储最优解信息
struct OptimalSolution {
    string instance_name;
    string depot_position;
    int drone_count;
    double optimal_makespan;
};

// -----------------------------
// CSV escape
// -----------------------------
static std::string csvEscape(std::string s) {
    bool needQuote = false;
    for (char c : s) {
        if (c == '"') needQuote = true;
        if (c == ',' || c == '\n' || c == '\r') needQuote = true;
    }
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        if (c == '"') out += "\"\"";
        else out += c;
    }
    if (needQuote) return "\"" + out + "\"";
    return out;
}

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
            if (c == '"') inQuotes = true;
            else if (c == ',') { out.push_back(cur); cur.clear(); }
            else cur.push_back(c);
        }
    }
    out.push_back(cur);
    return out;
}

// -----------------------------
// join ints
// -----------------------------
static std::string joinInts(const std::vector<int>& v, const std::string& sep = "->") {
    std::string s;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) s += sep;
        s += std::to_string(v[i]);
    }
    return s;
}

// -----------------------------
// timestamp file name
// -----------------------------
static std::string timestampNow(const std::string& base, const std::string& ext) {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tmv{};
#ifdef _WIN32
    localtime_s(&tmv, &t);
#else
    localtime_r(&t, &tmv);
#endif
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
        tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
        tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
    return base + "_" + std::string(buf) + ext;
}

// -----------------------------
// Basic nint & distances (int rounded)
// -----------------------------
static inline int nint(double x) { return (int)std::floor(x + 0.5); }

static inline int euclideanDistanceInt(pair<double, double> a, pair<double, double> b) {
    double dx = a.first - b.first, dy = a.second - b.second;
    return nint(std::sqrt(dx * dx + dy * dy));
}
static inline int manhattanDistanceInt(pair<double, double> a, pair<double, double> b) {
    double v = std::abs(a.first - b.first) + std::abs(a.second - b.second);
    return nint(v);
}

// -----------------------------
// TSPLIB: place depot ("C" center or "L" left-bottom)
// -----------------------------
static void placeDepot(vector<pair<double, double>>& coords, const string& depot_position) {
    double min_x = 1e18, max_x = -1e18, min_y = 1e18, max_y = -1e18;
    for (auto& p : coords) {
        min_x = min(min_x, p.first); max_x = max(max_x, p.first);
        min_y = min(min_y, p.second); max_y = max(max_y, p.second);
    }
    double depot_x, depot_y;
    if (depot_position == "C") {
        depot_x = (max_x + min_x) / 2.0;
        depot_y = (max_y + min_y) / 2.0;
    }
    else { // "L"
        depot_x = min_x; depot_y = min_y;
    }
    coords.insert(coords.begin(), { depot_x, depot_y }); // depot=0
}

// -----------------------------
// select station: min avg drone distance
// -----------------------------
static int findDroneStationMinAvg(const vector<vector<double>>& drone_time) {
    int N = (int)drone_time.size();
    int best = 1;
    double best_val = 1e100;

    for (int i = 1; i < N; i++) {
        double sum = 0.0;
        for (int j = 1; j < N; j++) {
            if (i == j) continue;
            sum += drone_time[i][j];
        }
        double avg = sum / (N - 2);
        if (avg < best_val) {
            best_val = avg;
            best = i;
        }
    }
    return best;
}

// -----------------------------
// eligibility deterministic (your rule)
// -----------------------------
static void setupEligibilityDeterministic(
    const vector<pair<double, double>>& nodes,
    int depot, int station,
    int K_percent,
    vector<bool>& is_truck_only
) {
    int N = (int)nodes.size();
    double K = K_percent / 100.0;
    double L = 0.30;

    int customers = N - 1;
    int target_drone = (int)std::ceil(customers * K);

    is_truck_only.assign(N, true);
    is_truck_only[depot] = true;
    is_truck_only[station] = true;

    int denom = (int)std::floor(1.0 / ((1.0 - K) * L));
    if (denom < 1) denom = 1;

    vector<bool> is_weight_block(N, false);
    for (int v = 1; v < N; ++v) {
        if (v == station) continue;
        if (v % denom == 0) is_weight_block[v] = true;
    }

    vector<pair<int, int>> dist_list;
    dist_list.reserve(N - 1);
    for (int v = 1; v < N; ++v) {
        if (v == station) continue;
        double dx = nodes[v].first - nodes[depot].first;
        double dy = nodes[v].second - nodes[depot].second;
        int d = nint(std::sqrt(dx * dx + dy * dy));
        dist_list.push_back({ d, v });
    }
    sort(dist_list.begin(), dist_list.end());

    int cnt = 0;
    for (auto& pr : dist_list) {
        int v = pr.second;
        if (is_weight_block[v]) continue;
        if (cnt >= target_drone) break;
        is_truck_only[v] = false;
        cnt++;
    }

    is_truck_only[depot] = true;
    is_truck_only[station] = true;
}

// -----------------------------
// TSPLIB -> MTSPDSGraph
// -----------------------------
static MTSPDSGraph buildGraphFromTSPLIB(
    const string& filename,
    int drone_count,
    double speed_ratio,
    double drone_range,
    const string& depot_position,
    int K_percent,
    int KN = 1,
    int C = 1
) {
    ifstream file(filename);
    if (!file) throw runtime_error("Cannot open tsp: " + filename);

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
            coords[id - 1] = { x, y };
        }
    }
    file.close();

    placeDepot(coords, depot_position);
    int N = (int)coords.size();

    vector<vector<double>> truck_time(N, vector<double>(N, 0.0));
    vector<vector<double>> drone_time(N, vector<double>(N, 0.0));
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            truck_time[i][j] = (double)manhattanDistanceInt(coords[i], coords[j]);
            drone_time[i][j] = (double)euclideanDistanceInt(coords[i], coords[j]) * speed_ratio;
        }
    }

    int depot = 0;
    int station = findDroneStationMinAvg(drone_time);

    vector<bool> is_truck_only;
    setupEligibilityDeterministic(coords, depot, station, K_percent, is_truck_only);

    MTSPDSGraph g;
    g.initialize(N);
    g.nodes = coords;
    g.depot = depot;
    g.truck_time = std::move(truck_time);
    g.drone_time = std::move(drone_time);

    g.speed_ratio = speed_ratio;
    g.max_roundtrip_dist = drone_range;
    g.KN = KN;
    g.C = C;
    g.DN = drone_count;

    g.is_station.assign(N, false);
    g.is_customer.assign(N, false);
    g.is_truck_only.assign(N, true);

    g.is_station[station] = true;
    for (int v = 0; v < N; ++v) {
        if (v == depot) continue;
        if (v == station) continue;
        g.is_customer[v] = true;
        g.is_truck_only[v] = is_truck_only[v];
    }
    g.is_truck_only[depot] = true;
    g.is_truck_only[station] = true;

    g.rebuildSets();
    return g;
}

// -----------------------------
// origin CSV -> MTSPDSGraph
// -----------------------------
struct CsvNodeRec { int id = -1; double x = 0.0, y = 0.0; int flag = 0; };

static inline string trimCopy(string s) {
    auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
    return s;
}

static bool parseCsvLineLast4(const string& line, CsvNodeRec& out) {
    vector<string> cells; cells.reserve(8);
    stringstream ss(line);
    string cell;
    while (getline(ss, cell, ',')) cells.push_back(trimCopy(cell));
    if ((int)cells.size() < 4) return false;
    int base = (int)cells.size() - 4;
    try {
        out.id = stoi(cells[base + 0]);
        out.x = stod(cells[base + 1]);
        out.y = stod(cells[base + 2]);
        out.flag = stoi(cells[base + 3]);
        return true;
    }
    catch (...) {
        return false;
    }
}

static MTSPDSGraph buildGraphFromOriginCsv(
    const string& csvPath,
    int drone_count,
    double speed_ratio,
    double drone_range,
    int KN = 1,
    int C = 1
) {
    ifstream in(csvPath);
    if (!in) throw runtime_error("Cannot open csv: " + csvPath);

    vector<CsvNodeRec> recs;
    string line;
    while (getline(in, line)) {
        line = trimCopy(line);
        if (line.empty()) continue;
        CsvNodeRec r;
        if (!parseCsvLineLast4(line, r)) continue;
        if (r.id < 0) continue;
        recs.push_back(r);
    }
    in.close();
    if (recs.empty()) throw runtime_error("CSV has no valid rows: " + csvPath);

    int maxId = -1;
    for (auto& r : recs) maxId = max(maxId, r.id);
    int N = maxId + 1;

    vector<pair<double, double>> coords(N, { 0.0, 0.0 });
    vector<int> flags(N, 0);
    vector<char> seen(N, 0);
    for (auto& r : recs) {
        if (r.id >= 0 && r.id < N) {
            coords[r.id] = { r.x, r.y };
            flags[r.id] = r.flag;
            seen[r.id] = 1;
        }
    }

    int depot = 0;
    if (depot < 0 || depot >= N || !seen[depot])
        throw runtime_error("CSV cannot find depot id=0: " + csvPath);

    std::vector<int> station_nodes;
    station_nodes.reserve(16);

    for (int i = 0; i < N; ++i) {
        if (!seen[i]) continue;
        if (i == depot) continue;
        if (flags[i] == 2) station_nodes.push_back(i);
    }

    if (station_nodes.empty())
        throw runtime_error("CSV has no station flag=2: " + csvPath);


    vector<vector<double>> truck_time(N, vector<double>(N, 0.0));
    vector<vector<double>> drone_time(N, vector<double>(N, 0.0));
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            truck_time[i][j] = (double)manhattanDistanceInt(coords[i], coords[j]);
            drone_time[i][j] = (double)euclideanDistanceInt(coords[i], coords[j]) * speed_ratio;
        }
    }

    MTSPDSGraph g;
    g.initialize(N);
    g.nodes = coords;
    g.depot = depot;
    g.truck_time = std::move(truck_time);
    g.drone_time = std::move(drone_time);

    g.speed_ratio = speed_ratio;
    g.max_roundtrip_dist = drone_range;
    g.KN = KN;
    g.C = C;
    g.DN = drone_count;

    g.is_station.assign(N, false);
    g.is_customer.assign(N, false);
    g.is_truck_only.assign(N, true);

    // 1) 标记所有 station
    for (int s : station_nodes) {
        g.is_station[s] = true;
        g.is_truck_only[s] = true;   // 不是 customer，设 true/false 都行；设 true 更安全
        g.is_customer[s] = false;
    }

    // 2) 其余点（非 depot、非 station）才是 customer
    for (int v = 0; v < N; ++v) {
        if (v == depot) continue;
        if (!seen[v]) continue;
        if (g.is_station[v]) continue;   // 跳过所有 station

        g.is_customer[v] = true;
        g.is_truck_only[v] = (flags[v] != 0); // flag=0 => drone-eligible
    }

    // 3) depot 固定
    g.is_truck_only[depot] = true;
    g.is_customer[depot] = false;
    g.is_station[depot] = false;


    g.rebuildSets();
    return g;
}

// -----------------------------
// Murray folder -> MTSPDSGraph
// folder 内需要包含 nodes.csv / tau.csv / tauprime.csv / Cprime.csv
// -----------------------------
static std::vector<std::vector<double>> readMatrixCsv(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open: " + path);

    std::vector<std::vector<double>> M;
    std::string line;
    while (std::getline(in, line)) {
        line = trimCopy(line);
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string cell;
        std::vector<double> row;
        while (std::getline(ss, cell, ',')) {
            cell = trimCopy(cell);
            if (!cell.empty()) row.push_back(std::stod(cell));
        }
        if (!row.empty()) M.push_back(std::move(row));
    }
    return M;
}

// nodes.csv: 通常是 id,x,y,...；这里只取 id,x,y
static std::vector<std::pair<double, double>> readNodesXY(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open: " + path);

    std::vector<std::pair<double, double>> xy;
    std::string line;
    while (std::getline(in, line)) {
        line = trimCopy(line);
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string a, b, c;
        if (!std::getline(ss, a, ',')) continue;
        if (!std::getline(ss, b, ',')) continue;
        if (!std::getline(ss, c, ',')) continue;

        int id = std::stoi(trimCopy(a));
        double x = std::stod(trimCopy(b));
        double y = std::stod(trimCopy(c));

        if ((int)xy.size() <= id) xy.resize(id + 1);
        xy[id] = { x, y };
    }
    return xy;
}

// Cprime.csv: 可由无人机服务的候选客户集合
static std::vector<int> readCprime(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open: " + path);

    std::vector<int> Cprime;
    std::string line;
    while (std::getline(in, line)) {
        line = trimCopy(line);
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            cell = trimCopy(cell);
            if (!cell.empty()) Cprime.push_back(std::stoi(cell));
        }
    }
    std::sort(Cprime.begin(), Cprime.end());
    Cprime.erase(std::unique(Cprime.begin(), Cprime.end()), Cprime.end());
    return Cprime;
}

static bool isSquareMatrix(const std::vector<std::vector<double>>& M) {
    if (M.empty()) return false;
    int n = (int)M.size();
    for (const auto& row : M) {
        if ((int)row.size() != n) return false;
    }
    return true;
}

// Murray 常见有起点 depot=0 和终点 depot=c+1；若最后一个点与 depot 坐标相同，则删除终点副本
static void dropLastNodeAndMatrix(
    std::vector<std::pair<double, double>>& nodes,
    std::vector<std::vector<double>>& tau,
    std::vector<std::vector<double>>& taup)
{
    if (nodes.size() <= 2) return;
    nodes.pop_back();
    tau.pop_back();
    taup.pop_back();
    for (auto& row : tau) row.pop_back();
    for (auto& row : taup) row.pop_back();
}

static MTSPDSGraph buildGraphFromMurrayFolder(
    const std::string& folder,
    int drone_count,
    double drone_range_minutes,
    int KN = 1,
    int C = 1
) {
    auto nodes = readNodesXY((fs::path(folder) / "nodes.csv").string());
    auto tau = readMatrixCsv((fs::path(folder) / "tau.csv").string());
    auto taup = readMatrixCsv((fs::path(folder) / "tauprime.csv").string());
    auto Cprime = readCprime((fs::path(folder) / "Cprime.csv").string());

    if (!isSquareMatrix(tau) || !isSquareMatrix(taup)) {
        throw std::runtime_error("tau/tauprime not square matrices: " + folder);
    }
    if ((int)nodes.size() != (int)tau.size() || (int)nodes.size() != (int)taup.size()) {
        throw std::runtime_error("nodes size != tau/tauprime dimension: " + folder);
    }

    auto nearEq = [](double a, double b) { return std::fabs(a - b) <= 1e-9; };
    bool lastLooksLikeEndDepot =
        !nodes.empty() &&
        nearEq(nodes.front().first, nodes.back().first) &&
        nearEq(nodes.front().second, nodes.back().second);
    if (lastLooksLikeEndDepot) {
        dropLastNodeAndMatrix(nodes, tau, taup);
    }

    int N = (int)nodes.size();
    if (N <= 2) throw std::runtime_error("Murray instance too small: " + folder);

    std::unordered_set<int> Cset(Cprime.begin(), Cprime.end());

    auto roundTrip = [&](int s, int j)->double {
        return taup[s][j] + taup[j][s];
        };

    // 文献设置：station 取覆盖客户数最多的客户点
    int bestStation = 1;
    int bestCover = -1;
    for (int s = 1; s < N; ++s) {
        int cover = 0;
        for (int j : Cprime) {
            if (j <= 0 || j >= N || j == s) continue;
            if (roundTrip(s, j) <= drone_range_minutes + 1e-9) cover++;
        }
        if (cover > bestCover) {
            bestCover = cover;
            bestStation = s;
        }
    }

    MTSPDSGraph g;
    g.initialize(N);
    g.nodes = std::move(nodes);
    g.depot = 0;
    g.truck_time = std::move(tau);
    g.drone_time = std::move(taup);

    g.speed_ratio = 1.0;                  // Murray 已给定时间矩阵，不再乘速度比
    g.max_roundtrip_dist = drone_range_minutes;
    g.KN = KN;
    g.C = C;
    g.DN = drone_count;

    g.is_station.assign(N, false);
    g.is_customer.assign(N, false);
    g.is_truck_only.assign(N, true);

    g.is_station[bestStation] = true;

    for (int v = 0; v < N; ++v) {
        if (v == g.depot) {
            g.is_customer[v] = false;
            g.is_truck_only[v] = true;
            continue;
        }
        if (v == bestStation) {
            g.is_customer[v] = false;
            g.is_truck_only[v] = true;
            continue;
        }

        bool eligibleByWeight = (Cset.count(v) > 0);
        bool eligibleByRange = (roundTrip(bestStation, v) <= drone_range_minutes + 1e-9);
        bool droneEligible = eligibleByWeight && eligibleByRange;

        g.is_customer[v] = true;
        g.is_truck_only[v] = !droneEligible;
    }

    g.rebuildSets();
    return g;
}


// -----------------------------
// list .tsp/.csv / list Murray folders
// -----------------------------
static vector<string> listInstanceFiles(const fs::path& input_dir) {
    vector<string> files;
    for (const auto& entry : fs::directory_iterator(input_dir)) {
        if (!entry.is_regular_file()) continue;
        string ext = entry.path().extension().string();
        transform(ext.begin(), ext.end(), ext.begin(),
            [](unsigned char c) { return (char)tolower(c); });
        if (ext == ".tsp" || ext == ".csv") files.push_back(entry.path().string());
    }
    sort(files.begin(), files.end());
    return files;
}

static vector<string> listSubDirs(const fs::path& root) {
    vector<string> dirs;
    if (!fs::exists(root)) return dirs;

    for (const auto& entry : fs::directory_iterator(root)) {
        if (entry.is_directory()) dirs.push_back(entry.path().string());
    }
    sort(dirs.begin(), dirs.end());
    return dirs;
}

// -----------------------------
// Job & ResultRow
// -----------------------------
struct Job {
    std::string path;
    std::string depot_pos;
    int drone_count = 1;
    int run_id = 0;
    bool is_murray = false;
};

struct ResultRow {
    bool ok = true;
    std::string instance;
    std::string depot_pos;
    int drone_count = 0;
    int run_id = 0;
    uint32_t seed = 0;
    int nodes = 0;
    double makespan = 0.0;
    double time_sec = 0.0;
    std::string truck_route;
    std::string error;

    bool is_murray = false;
};

static inline std::string toLowerCopy(std::string s) {
    for (auto& ch : s) ch = (char)std::tolower((unsigned char)ch);
    return s;
}

static inline std::vector<std::string> splitByUnderscore(const std::string& s) {
    std::vector<std::string> tok;
    std::string cur;
    for (char c : s) {
        if (c == '_') { tok.push_back(cur); cur.clear(); }
        else cur.push_back(c);
    }
    tok.push_back(cur);
    return tok;
}

// token -> depot("C"/"L")，支持: 0/c/C => C,  l/L/1 => L
static inline bool decodeDepotToken(const std::string& token, std::string& outDepot) {
    std::string t = toLowerCopy(token);
    if (t == "0" || t == "c" || t == "center") { outDepot = "C"; return true; }
    if (t == "l" || t == "1" || t == "left") { outDepot = "L"; return true; }
    if (t == "c") { outDepot = "C"; return true; }
    if (t == "l") { outDepot = "L"; return true; }
    return false;
}

// 从 filename(stem) 解析出 baseName + depot
// 支持: att48_0_80.csv / att48_l_80.csv / kroA100_C_80.csv
// 规则：优先取“倒数第二段”为 depot token（因为最后一段通常是 K=80）
static inline bool parseBaseAndDepotFromFilename(const std::string& instance_filename,
    std::string& outBase,
    std::string& outDepot) {
    std::string stem = fs::path(instance_filename).stem().string(); // att48_0_80
    auto tok = splitByUnderscore(stem);
    if (tok.size() < 2) return false;

    // case A: *_<depot>_<K>  => depot = tok[tok.size()-2]
    if (tok.size() >= 3) {
        std::string dep;
        if (decodeDepotToken(tok[tok.size() - 2], dep)) {
            outDepot = dep;

            // base = tok[0 .. size-3] 连接
            outBase = tok[0];
            for (size_t i = 1; i + 2 < tok.size(); ++i) outBase += "_" + tok[i];
            return true;
        }
    }

    // case B: *_<depot> (没有K) => depot = last token
    {
        std::string dep;
        if (decodeDepotToken(tok.back(), dep)) {
            outDepot = dep;
            outBase = tok[0];
            for (size_t i = 1; i + 1 < tok.size(); ++i) outBase += "_" + tok[i];
            return true;
        }
    }

    return false;
}

static bool lookupOptimalMakespan(const map<string, OptimalSolution>& optimalSolutions,
    const std::string& instance_filename, // 例如 "att48_0_80.csv" / "eil101.tsp"
    const std::string& depot_pos,         // "L"/"C" (对 csv 可被 filename 覆盖)
    int drone_count,
    double& out_opt) {

    out_opt = std::numeric_limits<double>::infinity();

    // 1) 默认：按调用者传入 depot_pos + stem 当作 base
    std::string stem = fs::path(instance_filename).stem().string(); // "att48_0_80"
    std::string baseCandidate = stem;
    std::string depotCandidate = depot_pos;

    // 2) 如果文件名里能解析出 base+depot（att48_0_80 => att48 + C）
    std::string parsedBase, parsedDepot;
    if (parseBaseAndDepotFromFilename(instance_filename, parsedBase, parsedDepot)) {
        baseCandidate = parsedBase;
        depotCandidate = parsedDepot; // 0->C, l->L
    }

    // 3) 生成候选 key（按你 readOptimalSolutions 的 key 规则）
    //    key = instance_name + "_" + depot + "_" + drone_count
    std::vector<std::string> keys;
    keys.reserve(6);

    keys.push_back(baseCandidate + "_" + depotCandidate + "_" + std::to_string(drone_count));

    // fallback: 有时候 depot_pos 传入的也可能是对的
    if (depotCandidate != depot_pos) {
        keys.push_back(baseCandidate + "_" + depot_pos + "_" + std::to_string(drone_count));
    }

    // fallback: 极端情况：有人把 instance_name 写成 stem 的第一段（att48）
    //           或者直接是 stem（att48_0_80）
    {
        auto tok = splitByUnderscore(stem);
        if (!tok.empty()) {
            keys.push_back(tok[0] + "_" + depotCandidate + "_" + std::to_string(drone_count));
            if (depotCandidate != depot_pos)
                keys.push_back(tok[0] + "_" + depot_pos + "_" + std::to_string(drone_count));
        }
        keys.push_back(stem + "_" + depotCandidate + "_" + std::to_string(drone_count));
        if (depotCandidate != depot_pos)
            keys.push_back(stem + "_" + depot_pos + "_" + std::to_string(drone_count));
    }

    for (const auto& k : keys) {
        auto it = optimalSolutions.find(k);
        if (it != optimalSolutions.end()) {
            out_opt = it->second.optimal_makespan;
            return true;
        }
    }
    return false;
}



// -----------------------------
// Solve one job (in one process)
// -----------------------------
static ResultRow solveOneJob(
    const Job& job,
    const TwoPhaseParams& baseParams,
    double speed_ratio,
    double drone_range,
    double murray_drone_range_minutes,
    int K_percent,
    int KN,
    int C,
    const map<string, OptimalSolution>& optimalSolutions
) {
    ResultRow row;
    row.instance = fs::path(job.path).filename().string();
    row.depot_pos = job.depot_pos;
    row.drone_count = job.drone_count;
    row.run_id = job.run_id;
    row.is_murray = job.is_murray;

    auto t0 = std::chrono::high_resolution_clock::now();

    try {
        std::string seed_key =
            fs::path(job.path).stem().string() + "|" + job.depot_pos +
            "|D" + std::to_string(job.drone_count) + "|run" + std::to_string(job.run_id);
        row.seed = (uint32_t)(std::hash<std::string>{}(seed_key) & 0xFFFFFFFFu);


        std::string ext = fs::path(job.path).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(),
            [](unsigned char c) { return (char)std::tolower(c); });

        MTSPDSGraph g;
        if (job.is_murray) {
            g = buildGraphFromMurrayFolder(job.path, job.drone_count,
                murray_drone_range_minutes, KN, C);
        }
        else if (ext == ".tsp") {
            g = buildGraphFromTSPLIB(job.path, job.drone_count, speed_ratio, drone_range,
                job.depot_pos, K_percent, KN, C);
        }
        else {
            g = buildGraphFromOriginCsv(job.path, job.drone_count, speed_ratio, drone_range, KN, C);
        }
        row.nodes = (int)g.nodes.size();

        // ====== 查最优值（用于提前停止）======
        double opt_mk = std::numeric_limits<double>::infinity();
        bool has_opt = lookupOptimalMakespan(
            optimalSolutions,
            row.instance,          // 注意：这里 row.instance 是 filename()，例如 "eil101.tsp"
            row.depot_pos,         // "L"/"C"
            row.drone_count,
            opt_mk
        );

        double stop_target = has_opt ? opt_mk : std::numeric_limits<double>::infinity();

        TwoPhaseParams pp = baseParams;
        pp.seed = row.seed;

        TwoPhaseMatheuristic solver(g, pp);
        auto best = solver.solve(stop_target);

        row.makespan = best.makespan;
        if (!best.truck_routes.empty()) row.truck_route = joinInts(best.truck_routes[0], "->");

        row.ok = true;
    }
    catch (const std::exception& e) {
        row.ok = false;
        row.error = e.what();
        row.makespan = std::numeric_limits<double>::infinity();
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    row.time_sec = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0;
    return row;
}

static void writeRowCsv(std::ofstream& out, const ResultRow& r) {
    out << csvEscape(r.instance) << ","
        << csvEscape(r.depot_pos) << ","
        << r.drone_count << ","
        << r.run_id << ","
        << r.seed << ","
        << r.nodes << ","
        << (std::isfinite(r.makespan) ? r.makespan : -1) << ","
        << r.time_sec << ","
        << (r.ok ? 1 : 0) << ","
        << csvEscape(r.error) << ","
        << csvEscape(r.truck_route)
        << "\n";
}


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

int main() {
    // ========== Config ==========
    fs::path input_dir = fs::path("data") / "tsp_origan";
    // Murray 小实例根目录：每个子目录包含 nodes.csv / tau.csv / tauprime.csv / Cprime.csv
    fs::path murray_root = fs::path("data") / "Murray_Chu_2015_test_data" / "PDSTSP" / "temp";
    bool isSmallInstance = false; // true: Murray 小实例；false: TSPLIB / origin CSV
    int murray_sample_limit = 20;  // 0 表示全部；需要抽样时改成 20 等数量

    std::vector<int> drone_counts = { 1,3,5 }; // 1,3,5
    std::vector<std::string> depot_positions = { "L" }; // TSPLIB uses it; CSV ignores

    double speed_ratio = 0.5;
    double drone_range = std::numeric_limits<double>::infinity();
    double murray_drone_range_minutes = 30.0;
    int K_percent = 80;

    int KN = 1;//卡车数量
    int C = 3; //无人机站点数量

    TwoPhaseParams p;

    p.time_limit_sec = 60;
    p.rho = 50;
    p.delta = 50;
    p.pi_base = 1.25;
    p.phi = 0.2;
    p.lambda = 2.0;

    int WORKERS = 20;       // worker processes
    int NUM_REPEATS = 20;   // repeats per combination

    // Output dir
    fs::path out_dir = fs::path("results");
    fs::create_directories(out_dir);

    std::string baseName = timestampNow("two_phase_results", ".csv");
    fs::path final_csv = out_dir / baseName;

    fs::path optimal_file = fs::path("data") / "TSP-DS-OPT.txt";


    // 读取最优解文件（相对）
    auto optimalSolutions = readOptimalSolutions(optimal_file.string());

    // ========== Collect instances ==========

    std::vector<std::string> files;
    std::vector<std::string> murray_folders;

    if (!isSmallInstance) {
        files = listInstanceFiles(input_dir);
        if (files.empty()) {
            std::cerr << "No .tsp/.csv files in: " << input_dir << "\n";
            return 1;
        }
    }
    else {
        if (!fs::exists(murray_root)) {
            fs::path alt = fs::path("data") / "Murray_Chu_2015_test_data" / "PDSTSP" / "PDSTSP_20_customer_problems";
            if (fs::exists(alt)) murray_root = alt;
        }

        murray_folders = listSubDirs(murray_root);
        if (murray_sample_limit > 0 && (int)murray_folders.size() > murray_sample_limit) {
            std::mt19937 rng((unsigned)std::chrono::high_resolution_clock::now().time_since_epoch().count());
            std::shuffle(murray_folders.begin(), murray_folders.end(), rng);
            murray_folders.resize(murray_sample_limit);
            std::sort(murray_folders.begin(), murray_folders.end());
        }
        if (murray_folders.empty()) {
            std::cerr << "No Murray folders in: " << murray_root << "\n";
            return 1;
        }
    }

    // ========== Build jobs ==========
    std::vector<Job> jobs;
    size_t instance_count = isSmallInstance ? murray_folders.size() : files.size();
    jobs.reserve(instance_count * depot_positions.size() * drone_counts.size() * (size_t)NUM_REPEATS);

    for (int run_id = 0; run_id < NUM_REPEATS; ++run_id) {
        if (!isSmallInstance) {
            for (const auto& path : files) {
                for (const auto& depot_pos : depot_positions) {
                    for (int dc : drone_counts) {
                        jobs.push_back(Job{ path, depot_pos, dc, run_id, false });
                    }
                }
            }
        }
        else {
            for (const auto& folder : murray_folders) {
                for (int dc : drone_counts) {
                    jobs.push_back(Job{ folder, "MURRAY_H", dc, run_id, true });
                }
            }
        }
    }

    std::cout << "Instances=" << instance_count
        << " jobs=" << jobs.size()
        << " WORKERS=" << WORKERS
        << " repeats=" << NUM_REPEATS << "\n";

    // parent single thread
    omp_set_dynamic(0);
    omp_set_num_threads(1);

    // prevent hidden multi-thread libs
    setenv("OMP_NUM_THREADS", "1", 1);
    setenv("MKL_NUM_THREADS", "1", 1);
    setenv("OPENBLAS_NUM_THREADS", "1", 1);
    setenv("NUMEXPR_NUM_THREADS", "1", 1);

    // ========== Worker CSV paths ==========
    std::vector<fs::path> worker_paths;
    worker_paths.reserve(WORKERS);
    for (int w = 0; w < WORKERS; ++w) {
        fs::path pth = out_dir / (std::string("worker_") + std::to_string(w) + "_" + baseName);
        worker_paths.push_back(pth);
    }

    // ========== fork WORKERS ==========
    std::vector<pid_t> pids;
    pids.reserve(WORKERS);
    /*ResultRow r = solveOneJob(jobs[0], p, speed_ratio, drone_range, K_percent, KN, C,
        optimalSolutions);*/
    for (int w = 0; w < WORKERS; ++w) {
        pid_t pid = fork();
        if (pid == 0) {
            // ----- child -----
            omp_set_dynamic(0);
            omp_set_num_threads(1);

            setenv("OMP_NUM_THREADS", "1", 1);
            setenv("MKL_NUM_THREADS", "1", 1);
            setenv("OPENBLAS_NUM_THREADS", "1", 1);
            setenv("NUMEXPR_NUM_THREADS", "1", 1);

            std::ofstream wf(worker_paths[w].string());
            if (!wf.is_open()) {
                std::cerr << "Worker " << w << " cannot open: " << worker_paths[w] << "\n";
                _exit(2);
            }

            wf << "Instance,DepotPosition,DroneCount,RunId,Seed,Nodes,Makespan,TimeSec,OK,Error,TruckRoute\n";

            for (int i = 0; i < (int)jobs.size(); ++i) {
                if (i % WORKERS != w) continue;
                const Job& job = jobs[i];
                ResultRow r = solveOneJob(job, p, speed_ratio, drone_range,
                    murray_drone_range_minutes, K_percent, KN, C, optimalSolutions);
                writeRowCsv(wf, r);
            }

            wf.close();
            _exit(0);
        }

        if (pid > 0) pids.push_back(pid);
        else {
            perror("fork");
            break;
        }
    }

    // ========== parent waits ==========
    for (pid_t pid : pids) {
        int status = 0;
        waitpid(pid, &status, 0);
    }

    // ========== merge worker CSV -> final, and compute SUMMARY stats ==========
    std::ofstream out(final_csv.string());
    if (!out.is_open()) {
        std::cerr << "Cannot open final output: " << final_csv << "\n";
        return 3;
    }

    // final header
    out << "Instance,DepotPosition,DroneCount,RunId,Seed,Nodes,Makespan,TimeSec,OK,Error,TruckRoute\n";

    // 统计结构：按 (Instance, DroneCount) 聚合
    struct StatKeep {
        double best_mk = std::numeric_limits<double>::infinity();
        int best_count = 0;
        int ok_runs = 0;
        int total_runs = 0;
        std::vector<std::string> best_cells;

        double sum_time_ok = 0.0;        // 所有 OK 的时间和
        double sum_time_best = 0.0;      // 所有达到 best_mk 的时间和
    };


    std::unordered_map<std::string, StatKeep> statMap;
    statMap.reserve(2048);

    auto makeKey = [](const std::string& instance, const std::string& droneCountStr) {
        return instance + "|D=" + droneCountStr;
        };

    const double EPS = 1e-9;

    // append all rows + update statMap
    for (int w = 0; w < (int)worker_paths.size(); ++w) {
        std::ifstream in(worker_paths[w].string());
        if (!in.is_open()) continue;

        std::string line;
        bool first = true;
        while (std::getline(in, line)) {
            if (first) { first = false; continue; } // skip worker header
            if (line.empty()) continue;

            // write detail
            out << line << "\n";

            // parse
            auto cells = splitCsvRow(line);
            if (cells.size() < 11) continue;

            const std::string& instance = cells[0];
            const std::string& dcStr = cells[2];

            std::string key = makeKey(instance, dcStr);
            auto& st = statMap[key];
            st.total_runs++;

            int ok = 0;
            try { ok = std::stoi(cells[8]); }
            catch (...) { ok = 0; }
            if (ok != 1) continue;

            st.ok_runs++;

            double tsec = 0.0;
            try { tsec = std::stod(cells[7]); }  // TimeSec 在第 8 列(0-based index=7)
            catch (...) { tsec = 0.0; }
            st.sum_time_ok += tsec;


            double mk = std::numeric_limits<double>::infinity();
            try { mk = std::stod(cells[6]); }
            catch (...) { continue; }
            if (!std::isfinite(mk)) continue;

            if (mk + EPS < st.best_mk) {
                st.best_mk = mk;
                st.best_count = 1;
                st.best_cells = cells;
                st.sum_time_best = tsec;
            }
            else if (std::fabs(mk - st.best_mk) <= EPS) {
                st.best_count++;
                st.sum_time_best += tsec;
            }

        }
    }

    // SUMMARY block (加 BestCount + TotalRuns + OkRuns)
    out << "SUMMARY,BEST_PER_(Instance,DroneCount)\n";
    out << "Instance,DepotPosition,DroneCount,BestMakespan,BestCount,OkRuns,TotalRuns,AvgTimeAllOk,AvgTimeBest\n";


    struct Item {
        std::string instance;
        int dc = 0;
        StatKeep st;
    };
    std::vector<Item> items;
    items.reserve(statMap.size());

    for (auto& kv : statMap) {
        const auto& st = kv.second;
        // dc 从 key 里解析也行；这里用 best_cells 更稳（若全失败，best_cells 为空）
        int dc = 0;
        if (!st.best_cells.empty()) {
            try { dc = std::stoi(st.best_cells[2]); }
            catch (...) { dc = 0; }
            items.push_back(Item{ st.best_cells[0], dc, st });
        }
        else {
            // 全失败时，也要输出统计（best=inf, best_count=0）
            // 从 key 分割出 instance 和 dcStr
            auto pos = kv.first.rfind("|D=");
            std::string inst = (pos == std::string::npos) ? kv.first : kv.first.substr(0, pos);
            std::string dcStr = (pos == std::string::npos) ? "0" : kv.first.substr(pos + 3);
            try { dc = std::stoi(dcStr); }
            catch (...) { dc = 0; }
            items.push_back(Item{ inst, dc, st });
        }
    }

    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
        if (a.instance != b.instance) return a.instance < b.instance;
        return a.dc < b.dc;
        });

    for (const auto& it : items) {
        const auto& st = it.st;
        double avg_ok = (st.ok_runs > 0) ? (st.sum_time_ok / st.ok_runs) : -1.0;
        double avg_best = (st.best_count > 0) ? (st.sum_time_best / st.best_count) : -1.0;

        out << csvEscape(it.instance) << ","
            << "BEST" << ","
            << it.dc << ","
            << (std::isfinite(st.best_mk) ? st.best_mk : -1) << ","
            << st.best_count << ","
            << st.ok_runs << ","
            << st.total_runs << ","
            << avg_ok << ","
            << avg_best
            << "\n";

    }

    out.close();
    std::cout << "Saved final: " << final_csv << "\n";

    // delete worker temp files
    for (const auto& pth : worker_paths) {
        std::error_code ec;
        fs::remove(pth, ec);
    }

    return 0;
}
