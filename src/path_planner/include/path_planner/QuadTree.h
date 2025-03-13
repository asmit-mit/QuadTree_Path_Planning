#ifndef QUAD_TREE_H
#define QUAD_TREE_H

#include <algorithm>
#include <vector>

class QuadTreeNode {
public:
  int x, y;
  int width, height;
  int value;
  bool is_leaf;
  QuadTreeNode *children[4];

  QuadTreeNode(int x, int y, int width, int height, int value, bool is_leaf);
  ~QuadTreeNode();
};

class QuadTree {
private:
  std::vector<QuadTreeNode *> leaf_nodes;
  int max_depth;

  bool isHomogeneous(std::vector<std::vector<int>> &grid, int x, int y,
                     int width, int height);
  QuadTreeNode *buildRecursive(std::vector<std::vector<int>> &grid, int x,
                               int y, int width, int height, int depth);
  void insertRecursive(QuadTreeNode *node, int x, int y, int value, int depth);
  int queryRecursive(QuadTreeNode *node, int x, int y);
  bool areNodesAdjacent(QuadTreeNode *node1, QuadTreeNode *node2);
  QuadTreeNode *findLeafNode(QuadTreeNode *node, int x, int y);
  void collectLeafNodes(QuadTreeNode *node,
                        std::vector<QuadTreeNode *> &leaves);

public:
  QuadTreeNode *root;

  QuadTree(int max_depth);
  ~QuadTree();
  void build(std::vector<std::vector<int>> &grid);
  void insert(int x, int y, int value);
  int query(int x, int y);
  void updateLeafNodesList();
  std::vector<QuadTreeNode *> getAdjacentLeafNodes(int x, int y);
  int getNumLeaves();
};

#endif
