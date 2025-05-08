
#ifndef ROBO_PLANNER_WS_HIERARCHICAL_GRAPH_H
#define ROBO_PLANNER_WS_HIERARCHICAL_GRAPH_H

#include <iostream>

#include <eigen3/Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/labeled_graph.hpp>
#include <ostream>

#include <utils/node.h>

namespace perception {

enum NodeType {
  Default = -1,
  Termination = 0,
  FakeTermination = 1,
  Connection = 2,
  Branch_junction = 3,
  Confluence_junction = 4,
};


struct NodeData {
  cv::Point cvPt;
  NodeType type = Default;
  bool confirmed = true;
  bool removed = false;
  int dis_to_parent = 0;
  int parent_label = -1;
  int parent_junction_label = -1;
  int parent_branch_label = -1;
  int generate_time = -1;
  int conn_num = -1;
  int cluster_id = -1;
  int simplified_label = -1;
  std::vector<int> children_labels;
  std::unordered_set<int> father_labels;
  std::unordered_set<int> unconfirmed_father_labels;
  NodeMap<NodeSet> children_with_unprocessed_frontier;
};

struct Edge {
  std::string name;
  double miles;
  int related_child_label;
  int related_father_label;
  int lanes;
  bool created = true;
};

struct Graph {
  int weight;
  int height;
  int root_label;
};

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
                              NodeData, Edge, Graph>
    Map;

typedef boost::labeled_graph<Map, int> LabeledMap;

typedef boost::graph_traits<Map>::vertex_descriptor VertexDescriptor;
typedef boost::graph_traits<Map>::edge_descriptor EdgeDescriptor;

typedef boost::graph_traits<Map>::vertex_iterator VertexIter;
typedef boost::graph_traits<Map>::edge_iterator EdgeIter;

}

#endif