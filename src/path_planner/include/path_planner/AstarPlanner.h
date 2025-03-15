#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

class AstarPlanner {
private:
  class Node {
  public:
    int x, y;
    double g_cost;
    double f_cost;
    Node *parent;

    Node();
    Node(int x, int y);
    bool operator==(const Node &other) const;
    bool operator!=(const Node &other) const;
  };

  struct NodeHash {
    size_t operator()(const Node &node) const;
  };

  struct NodeEqual {
    bool operator()(const Node &a, const Node &b) const;
  };

  struct Comparator {
    bool operator()(const Node *a, const Node *b) const;
  };

  std::vector<std::vector<int>> grid;
  int height;
  int width;
  const int directions[8][2] = {{0, 1},   {1, 0},  {0, -1}, {-1, 0},
                                {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};

  double heuristic(const Node *a, const Node *b);
  bool isValid(int x, int y);

public:
  AstarPlanner(std::vector<std::vector<int>> grid);
  std::vector<std::pair<int, int>> plan(int start_x, int start_y, int end_x,
                                        int end_y);
};

#endif
