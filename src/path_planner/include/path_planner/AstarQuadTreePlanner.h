#ifndef ASTAR_QUAD_TREE_PLANNER_H
#define ASTAR_QUAD_TREE_PLANNER_H

#include <algorithm>
#include <climits>
#include <cmath>
#include <functional>
#include <quadtrees/QuadTree.h>
#include <queue>
#include <unordered_map>
#include <vector>

class AstarQuadTreePlanner {
private:
  QuadTree *graph;

  class Node {
  public:
    QuadTreeNode *tree_node;
    double g_cost;
    double f_cost;
    Node *parent;

    Node();
    Node(QuadTreeNode *tree_node);
    bool operator==(const Node &other) const;
    bool operator!=(const Node &other) const;
  };

  struct NodePtrHash {
    std::size_t operator()(const Node *node) const;
  };

  struct NodePtrEqual {
    bool operator()(const Node *lhs, const Node *rhs) const;
  };

  struct Comparator {
    bool operator()(const Node *a, const Node *b) const;
  };

  bool isValid(int x, int y);
  double heuristic(int a_x, int a_y, int b_x, int b_y);
  bool nodeContainsPoint(Node *node, int x, int y);

public:
  AstarQuadTreePlanner();
  AstarQuadTreePlanner(QuadTree *graph);
  void setGraph(QuadTree *graph);
  std::vector<std::pair<int, int>> plan(int start_x, int start_y, int end_x,
                                        int end_y);
};

#endif // ASTAR_QUAD_TREE_PLANNER_H
