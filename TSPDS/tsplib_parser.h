#pragma once
#include "rsp_graph.h"
#include <string>

RSPGraph parseTSPLIB(const std::string& filename);

Solution parse_lkh_tour(const std::string& tour_file, const RSPGraph& graph);
