#include "path_planner/QuadTree.h"
#include <algorithm>

QuadTreeNode::QuadTreeNode(int x, int y, int width, int height, int value,
                           bool is_leaf) {
  this->x = x;
  this->y = y;
  this->width = width;
  this->height = height;
  this->value = value;
  this->is_leaf = is_leaf;

  for (int i = 0; i < 4; i++) {
    children[i] = nullptr;
  }
}

QuadTreeNode::~QuadTreeNode() {
  for (auto child : children) {
    delete child;
  }
}

QuadTree::QuadTree(int max_depth) {
  this->max_depth = max_depth;
  root = nullptr;
}

QuadTree::~QuadTree() { delete root; }

bool QuadTree::isHomogeneous(std::vector<std::vector<int>> &grid, int x, int y,
                             int width, int height) {
  int value = grid[y][x];
  for (int i = y; i < y + height; i++) {
    for (int j = x; j < x + width; j++) {
      if (grid[i][j] != value) {
        return false;
      }
    }
  }
  return true;
}

int QuadTree::getNumLeaves() { return leaf_nodes.size(); }

QuadTreeNode *QuadTree::buildRecursive(std::vector<std::vector<int>> &grid,
                                       int x, int y, int width, int height,
                                       int depth) {
  if (depth >= max_depth || isHomogeneous(grid, x, y, width, height)) {
    QuadTreeNode *leafNode =
        new QuadTreeNode(x, y, width, height, grid[y][x], true);
    leaf_nodes.push_back(leafNode);
    return leafNode;
  }

  QuadTreeNode *node = new QuadTreeNode(x, y, width, height, -2, false);

  if (width == 1 && height > 1) {
    int half_height = height / 2;
    int remaining_height = height - half_height;

    node->children[0] =
        buildRecursive(grid, x, y, width, half_height, depth + 1);
    node->children[1] = buildRecursive(grid, x, y + half_height, width,
                                       remaining_height, depth + 1);
  } else if (height == 1 && width > 1) {
    int half_width = width / 2;
    int remaining_width = width - half_width;

    node->children[0] =
        buildRecursive(grid, x, y, half_width, height, depth + 1);
    node->children[1] = buildRecursive(grid, x + half_width, y, remaining_width,
                                       height, depth + 1);
  } else if (width > 1 && height > 1) {
    int new_width = width / 2;
    int new_height = height / 2;
    int remaining_width = width - new_width;
    int remaining_height = height - new_height;

    node->children[0] =
        buildRecursive(grid, x, y, new_width, new_height, depth + 1);
    node->children[1] = buildRecursive(grid, x + new_width, y, remaining_width,
                                       new_height, depth + 1);
    node->children[2] = buildRecursive(grid, x, y + new_height, new_width,
                                       remaining_height, depth + 1);
    node->children[3] =
        buildRecursive(grid, x + new_width, y + new_height, remaining_width,
                       remaining_height, depth + 1);
  } else {
    delete node;
    QuadTreeNode *leafNode =
        new QuadTreeNode(x, y, width, height, grid[y][x], true);
    leaf_nodes.push_back(leafNode);
    return leafNode;
  }

  return node;
}

void QuadTree::build(std::vector<std::vector<int>> &grid) {
  if (grid.empty() || grid[0].empty()) {
    root = nullptr;
    return;
  }

  int height = grid.size();
  int width = grid[0].size();

  delete root;
  root = buildRecursive(grid, 0, 0, width, height, 0);
}

void QuadTree::insertRecursive(QuadTreeNode *node, int x, int y, int value,
                               int depth) {
  if (x < node->x || y < node->y || y >= node->y + node->height ||
      x >= node->x + node->width)
    return;

  if (node->is_leaf || depth >= max_depth) {
    if (node->is_leaf && node->value != value) {
      if (node->width == 1 && node->height > 1) {
        leaf_nodes.erase(
            std::remove(leaf_nodes.begin(), leaf_nodes.end(), node),
            leaf_nodes.end());
        node->is_leaf = false;

        int half_height = node->height / 2;
        int remaining_height = node->height - half_height;

        node->children[0] = new QuadTreeNode(node->x, node->y, node->width,
                                             half_height, node->value, true);
        node->children[1] =
            new QuadTreeNode(node->x, node->y + half_height, node->width,
                             remaining_height, node->value, true);

        leaf_nodes.push_back(node->children[0]);
        leaf_nodes.push_back(node->children[1]);

        node->value = -2;
        insertRecursive(node, x, y, value, depth);
      } else if (node->height == 1 && node->width > 1) {
        leaf_nodes.erase(
            std::remove(leaf_nodes.begin(), leaf_nodes.end(), node),
            leaf_nodes.end());
        node->is_leaf = false;

        int half_width = node->width / 2;
        int remaining_width = node->width - half_width;

        node->children[0] = new QuadTreeNode(node->x, node->y, half_width,
                                             node->height, node->value, true);
        node->children[1] =
            new QuadTreeNode(node->x + half_width, node->y, remaining_width,
                             node->height, node->value, true);

        leaf_nodes.push_back(node->children[0]);
        leaf_nodes.push_back(node->children[1]);

        node->value = -2;
        insertRecursive(node, x, y, value, depth);
      } else if (node->width > 1 && node->height > 1) {
        leaf_nodes.erase(
            std::remove(leaf_nodes.begin(), leaf_nodes.end(), node),
            leaf_nodes.end());
        node->is_leaf = false;

        int new_width = node->width / 2;
        int new_height = node->height / 2;
        int remaining_width = node->width - new_width;
        int remaining_height = node->height - new_height;

        node->children[0] = new QuadTreeNode(node->x, node->y, new_width,
                                             new_height, node->value, true);
        node->children[1] =
            new QuadTreeNode(node->x + new_width, node->y, remaining_width,
                             new_height, node->value, true);
        node->children[2] =
            new QuadTreeNode(node->x, node->y + new_height, new_width,
                             remaining_height, node->value, true);
        node->children[3] = new QuadTreeNode(
            node->x + new_width, node->y + new_height, remaining_width,
            remaining_height, node->value, true);

        leaf_nodes.push_back(node->children[0]);
        leaf_nodes.push_back(node->children[1]);
        leaf_nodes.push_back(node->children[2]);
        leaf_nodes.push_back(node->children[3]);

        node->value = -2;
        insertRecursive(node, x, y, value, depth);
      } else {
        node->value = value;
      }
    }
    return;
  }

  if (node->width == 1 && node->height > 1) {
    int mid_y = node->y + node->height / 2;
    int childIndex = (y < mid_y) ? 0 : 1;
    insertRecursive(node->children[childIndex], x, y, value, depth + 1);
  } else if (node->height == 1 && node->width > 1) {
    int mid_x = node->x + node->width / 2;
    int childIndex = (x < mid_x) ? 0 : 1;
    insertRecursive(node->children[childIndex], x, y, value, depth + 1);
  } else {
    int mid_x = node->x + node->width / 2;
    int mid_y = node->y + node->height / 2;

    int quadrant;
    if (x < mid_x) {
      if (y < mid_y) {
        quadrant = 0;
      } else {
        quadrant = 2;
      }
    } else {
      if (y < mid_y) {
        quadrant = 1;
      } else {
        quadrant = 3;
      }
    }

    insertRecursive(node->children[quadrant], x, y, value, depth + 1);
  }
}

void QuadTree::insert(int x, int y, int value) {
  if (!root)
    return;

  insertRecursive(root, x, y, value, 0);
}

int QuadTree::queryRecursive(QuadTreeNode *node, int x, int y) {
  if (x < node->x || y < node->y || x >= node->x + node->width ||
      y >= node->y + node->height) {
    return -1;
  }

  if (node->is_leaf) {
    return node->value;
  }

  if (node->width == 1 && node->height > 1) {
    int mid_y = node->y + node->height / 2;
    int childIndex = (y < mid_y) ? 0 : 1;
    return queryRecursive(node->children[childIndex], x, y);
  } else if (node->height == 1 && node->width > 1) {
    int mid_x = node->x + node->width / 2;
    int childIndex = (x < mid_x) ? 0 : 1;
    return queryRecursive(node->children[childIndex], x, y);
  } else {
    int mid_x = node->x + node->width / 2;
    int mid_y = node->y + node->height / 2;

    int quadrant;
    if (x < mid_x) {
      if (y < mid_y) {
        quadrant = 0;
      } else {
        quadrant = 2;
      }
    } else {
      if (y < mid_y) {
        quadrant = 1;
      } else {
        quadrant = 3;
      }
    }

    return queryRecursive(node->children[quadrant], x, y);
  }
}

int QuadTree::query(int x, int y) {
  if (!root)
    return -1;

  return queryRecursive(root, x, y);
}

bool QuadTree::areNodesAdjacent(QuadTreeNode *node1, QuadTreeNode *node2) {
  bool horizontallyAdjacent = (node1->x + node1->width == node2->x) ||
                              (node2->x + node2->width == node1->x);
  bool verticallyAdjacent = (node1->y + node1->height == node2->y) ||
                            (node2->y + node2->height == node1->y);

  bool shareHorizontalSpace =
      std::max(node1->x, node2->x) <
      std::min(node1->x + node1->width, node2->x + node2->width);
  bool shareVerticalSpace =
      std::max(node1->y, node2->y) <
      std::min(node1->y + node1->height, node2->y + node2->height);

  return (horizontallyAdjacent && shareVerticalSpace) ||
         (verticallyAdjacent && shareHorizontalSpace);
}

QuadTreeNode *QuadTree::findLeafNode(QuadTreeNode *node, int x, int y) {
  if (!node || x < node->x || y < node->y || x >= node->x + node->width ||
      y >= node->y + node->height)
    return nullptr;

  if (node->is_leaf)
    return node;

  if (node->width == 1 && node->height > 1) {
    int mid_y = node->y + node->height / 2;
    int childIndex = (y < mid_y) ? 0 : 1;
    return findLeafNode(node->children[childIndex], x, y);
  } else if (node->height == 1 && node->width > 1) {
    int mid_x = node->x + node->width / 2;
    int childIndex = (x < mid_x) ? 0 : 1;
    return findLeafNode(node->children[childIndex], x, y);
  } else {
    int mid_x = node->x + node->width / 2;
    int mid_y = node->y + node->height / 2;

    int quadrant;
    if (x < mid_x) {
      quadrant = (y < mid_y) ? 0 : 2;
    } else {
      quadrant = (y < mid_y) ? 1 : 3;
    }

    return findLeafNode(node->children[quadrant], x, y);
  }
}

void QuadTree::updateLeafNodesList() {
  leaf_nodes.clear();
  collectLeafNodes(root, leaf_nodes);
}

void QuadTree::collectLeafNodes(QuadTreeNode *node,
                                std::vector<QuadTreeNode *> &leaves) {
  if (!node)
    return;

  if (node->is_leaf) {
    leaves.push_back(node);
    return;
  }

  for (int i = 0; i < 4; i++) {
    if (node->children[i])
      collectLeafNodes(node->children[i], leaves);
  }
}

std::vector<QuadTreeNode *> QuadTree::getAdjacentLeafNodes(int x, int y) {
  QuadTreeNode *targetNode = findLeafNode(root, x, y);
  if (!targetNode || !targetNode->is_leaf)
    return {};

  std::vector<QuadTreeNode *> adjacentNodes;

  for (QuadTreeNode *node : leaf_nodes) {
    if (node != targetNode && areNodesAdjacent(node, targetNode)) {
      adjacentNodes.push_back(node);
    }
  }

  return adjacentNodes;
}
