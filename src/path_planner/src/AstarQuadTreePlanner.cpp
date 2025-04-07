#include "path_planner/AstarQuadTreePlanner.h"
#include "quadtrees/QuadTree.h"

AstarQuadTreePlanner::Node::Node() {
  this->tree_node = nullptr;
  this->parent = nullptr;
  this->g_cost = INT_MAX;
  this->f_cost = INT_MAX;
}

AstarQuadTreePlanner::Node::Node(QuadTreeNode *tree_node) {
  this->tree_node = tree_node;
  this->parent = nullptr;
  this->g_cost = INT_MAX;
  this->f_cost = INT_MAX;
}

bool AstarQuadTreePlanner::Node::operator==(const Node &other) const {
  return this->tree_node == other.tree_node;
}

bool AstarQuadTreePlanner::Node::operator!=(const Node &other) const {
  return !(*this == other);
}

std::size_t
AstarQuadTreePlanner::NodePtrHash::operator()(const Node *node) const {
  return std::hash<QuadTreeNode *>{}(node->tree_node);
}

bool AstarQuadTreePlanner::NodePtrEqual::operator()(const Node *lhs,
                                                    const Node *rhs) const {
  return lhs->tree_node == rhs->tree_node;
}

bool AstarQuadTreePlanner::Comparator::operator()(const Node *a,
                                                  const Node *b) const {
  if (a->f_cost == b->f_cost)
    return a->g_cost > b->g_cost;
  return a->f_cost > b->f_cost;
}

bool AstarQuadTreePlanner::isValid(int x, int y) {
  bool value_check = (graph->query(x, y) != 100);
  bool bounds_check = (x >= graph->root->x && y >= graph->root->y &&
                       x < graph->root->x + graph->root->width &&
                       y < graph->root->y + graph->root->height);
  return (value_check && bounds_check);
}

double AstarQuadTreePlanner::heuristic(int a_x, int a_y, int b_x, int b_y) {
  double dx = a_x - b_x;
  double dy = a_y - b_y;
  return std::sqrt(dx * dx + dy * dy);
}

bool AstarQuadTreePlanner::nodeContainsPoint(Node *node, int x, int y) {
  if (!node)
    return false;
  return (x >= node->tree_node->x &&
          x < node->tree_node->x + node->tree_node->width &&
          y >= node->tree_node->y &&
          y < node->tree_node->y + node->tree_node->height);
}

AstarQuadTreePlanner::AstarQuadTreePlanner() { this->graph = nullptr; }

AstarQuadTreePlanner::AstarQuadTreePlanner(QuadTree *graph) {
  this->graph = graph;
}

void AstarQuadTreePlanner::setGraph(QuadTree *graph) { this->graph = graph; }

std::vector<std::pair<int, int>>
AstarQuadTreePlanner::plan(int start_x, int start_y, int end_x, int end_y) {
  std::vector<std::pair<int, int>> path;
  if (!graph || !isValid(start_x, start_y) || !isValid(end_x, end_y)) {
    return path;
  }

  int width = graph->root->width;
  int height = graph->root->height;
  int offset_x = graph->root->x;
  int offset_y = graph->root->y;

  std::priority_queue<Node *, std::vector<Node *>, Comparator> pq;
  std::vector<Node *> nodes_grid(width * height, nullptr);

  QuadTreeNode *start_tree_node =
      graph->findLeafNode(graph->root, start_x, start_y);
  if (!start_tree_node)
    return path;

  Node *start_node = new Node(start_tree_node);
  start_node->g_cost = 0;
  start_node->f_cost = heuristic(start_x, start_y, end_x, end_y);

  pq.push(start_node);
  int start_idx =
      (start_tree_node->y - offset_y) * width + (start_tree_node->x - offset_x);
  nodes_grid[start_idx] = start_node;

  while (!pq.empty()) {
    Node *curr = pq.top();
    pq.pop();

    int curr_x = curr->tree_node->x;
    int curr_y = curr->tree_node->y;
    int curr_idx = (curr_y - offset_y) * width + (curr_x - offset_x);

    if (nodes_grid[curr_idx] != curr) {
      continue;
    }

    if (nodeContainsPoint(curr, end_x, end_y)) {
      while (curr != nullptr) {
        int curr_center_x = curr->tree_node->x + curr->tree_node->width / 2.0;
        int curr_center_y = curr->tree_node->y + curr->tree_node->height / 2.0;
        path.push_back(std::make_pair(curr_center_x, curr_center_y));
        curr = curr->parent;
      }

      std::reverse(path.begin(), path.end());
      path.push_back(std::make_pair(end_x, end_y));

      for (Node *node : nodes_grid) {
        delete node;
      }

      path[0] = std::make_pair(start_x, start_y);
      return path;
    }

    std::vector<QuadTreeNode *> neighbor_tree_nodes =
        graph->getAdjacentLeafNodes(curr_x, curr_y);

    for (auto tree_node : neighbor_tree_nodes) {
      int nx = tree_node->x;
      int ny = tree_node->y;
      int neighbor_idx = (ny - offset_y) * width + (nx - offset_x);

      if (tree_node->value == 100) {
        continue;
      }

      double curr_center_x = curr_x + curr->tree_node->width / 2.0;
      double curr_center_y = curr_y + curr->tree_node->height / 2.0;
      double next_center_x = nx + tree_node->width / 2.0;
      double next_center_y = ny + tree_node->height / 2.0;

      double movement_cost = std::sqrt(pow(next_center_x - curr_center_x, 2) +
                                       pow(next_center_y - curr_center_y, 2));
      double new_g_cost = curr->g_cost + movement_cost;

      Node *neighbor = nodes_grid[neighbor_idx];

      if (neighbor == nullptr || new_g_cost < neighbor->g_cost) {
        if (neighbor == nullptr) {
          neighbor = new Node(tree_node);
          nodes_grid[neighbor_idx] = neighbor;
        }

        neighbor->parent = curr;
        neighbor->g_cost = new_g_cost;
        neighbor->f_cost = new_g_cost + heuristic(nx, ny, end_x, end_y);
        pq.push(neighbor);
      }
    }
  }

  for (Node *node : nodes_grid) {
    delete node;
  }

  return path;
}
