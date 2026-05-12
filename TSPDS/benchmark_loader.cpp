#include "benchmark_loader.h"
#include <fstream>
#include <sstream>
#include <algorithm>

bool BenchmarkKey::operator==(const BenchmarkKey& other) const {
    return filename == other.filename && alpha == other.alpha;
}

namespace std {
    size_t hash<BenchmarkKey>::operator()(const BenchmarkKey& k) const {
        size_t h1 = hash<string>{}(k.filename);
        size_t h2 = hash<int>{}(k.alpha);
        return h1 ^ (h2 << 1);
    }
}

void load_benchmark_data(const std::string& filepath,
    std::unordered_map<BenchmarkKey, double>& benchmark_map)
{
    using namespace std;

    ifstream file(filepath);
    string line;
    while (getline(file, line)) {
        replace(line.begin(), line.end(), ',', ' ');
        istringstream iss(line);
        string filename;
        int alpha;
        double opt;
        if (iss >> filename >> alpha >> opt) {
            benchmark_map[{filename, alpha}] = opt;
        }
    }
}