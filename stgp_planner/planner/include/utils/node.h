
#ifndef ROBO_PLANNER_WS_NODE_H
#define ROBO_PLANNER_WS_NODE_H

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace utils {
using Node = int;

using NodeSet = std::unordered_set<Node>;
template <typename T> using NodeMap = std::unordered_map<Node, T>;
using NodeQueue = std::vector<Node>;

struct PathWithDistance {
  int distance=0;
  NodeQueue path;
  NodeSet frontiers;
};

using MissedBranchMap = std::unordered_map<Node, PathWithDistance>;


}

#endif

