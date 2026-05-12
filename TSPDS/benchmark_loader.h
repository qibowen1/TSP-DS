#pragma once
#include <string>
#include <unordered_map>

struct BenchmarkKey {
    std::string filename;
    int alpha;

    bool operator==(const BenchmarkKey& other) const;
};

namespace std {
    template<> struct hash<BenchmarkKey> {
        size_t operator()(const BenchmarkKey& k) const;
    };
}

void load_benchmark_data(const std::string& filepath,
    std::unordered_map<BenchmarkKey, double>& benchmark_map);
