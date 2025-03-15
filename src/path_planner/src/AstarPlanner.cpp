#include "path_planner/AstarPlanner.h"

AstarPlanner::Node::Node()
    : x(0), y(0), g_cost(std::numeric_limits<double>::max()),
      f_cost(std::numeric_limits<double>::max()), parent(nullptr) {}

AstarPlanner::Node::Node(int x, int y)
    : x(x), y(y), g_cost(std::numeric_limits<double>::max()),
      f_cost(std::numeric_limits<double>::max()), parent(nullptr) {}

bool AstarPlanner::Node::operator==(const Node &other) const {
  return x == other.x && y == other.y;
}

bool AstarPlanner::Node::operator!=(const Node &other) const {
  return x != other.x || y != other.y;
}

size_t AstarPlanner::NodeHash::operator()(const Node &node) const {
  return std::hash<int>()(node.x) ^ std::hash<int>()(node.y);
}

bool AstarPlanner::NodeEqual::operator()(const Node &a, const Node &b) const {
  return a.x == b.x && a.y == b.y;
}

bool AstarPlanner::Comparator::operator()(const Node *a, const Node *b) const {
  if (a->f_cost == b->f_cost)
    return a->g_cost > b->g_cost;
  return a->f_cost > b->f_cost;
}

AstarPlanner::AstarPlanner(std::vector<std::vector<int>> grid) {
  this->grid = grid;
  this->height = grid.size();
  this->width = grid[0].size();
}

double AstarPlanner::heuristic(const Node *a, const Node *b) {
  double dx = a->x - b->x;
  double dy = a->y - b->y;
  return std::sqrt(dx * dx + dy * dy);
}

bool AstarPlanner::isValid(int x, int y) {
  return (x >= 0 && x < width && y >= 0 && y < height && grid[y][x] != 100);
}

std::vector<std::pair<int, int>> AstarPlanner::plan(int start_x, int start_y,
                                                    int end_x, int end_y) {
  std::vector<std::pair<int, int>> path;
  if (!isValid(start_x, start_y) || !isValid(end_x, end_y)) {
    return path;
  }

  Node *start_node = new Node(start_x, start_y);
  Node *end_node = new Node(end_x, end_y);
  start_node->g_cost = 0;
  start_node->f_cost = heuristic(start_node, end_node);

  std::priority_queue<Node *, std::vector<Node *>, Comparator> pq;
  pq.push(start_node);
  std::unordered_map<int, std::unordered_map<int, Node *>> parent;
  std::unordered_map<int, std::unordered_map<int, bool>> visited;
  parent[start_x][start_y] = start_node;

  while (!pq.empty()) {
    Node *current = pq.top();
    pq.pop();

    if (current->x == end_x && current->y == end_y) {
      while (current != nullptr) {
        path.push_back(std::make_pair(current->x, current->y));
        current = current->parent;
      }
      std::reverse(path.begin(), path.end());

      for (auto &row : parent) {
        for (auto &col : row.second) {
          delete col.second;
        }
      }
      return path;
    }

    visited[current->x][current->y] = true;

    for (int i = 0; i < 8; i++) {
      int nx = current->x + directions[i][0];
      int ny = current->y + directions[i][1];

      if (!isValid(nx, ny) || visited[nx][ny]) {
        continue;
      }

      double movement_cost = (i < 4) ? 1.0 : 1.414;
      double new_g_cost = current->g_cost + movement_cost;
      Node *neighbor;

      if (parent[nx].find(ny) == parent[nx].end()) {
        neighbor = new Node(nx, ny);
        parent[nx][ny] = neighbor;
      } else {
        neighbor = parent[nx][ny];
      }

      if (new_g_cost < neighbor->g_cost) {
        neighbor->parent = current;
        neighbor->g_cost = new_g_cost;
        neighbor->f_cost = new_g_cost + heuristic(neighbor, end_node);
        pq.push(neighbor);
      }
    }
  }

  for (auto &row : parent) {
    for (auto &col : row.second) {
      delete col.second;
    }
  }
  return path;
}
