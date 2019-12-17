#ifndef TREE_NODE_H
#define TREE_NODE_H
#include <iostream>
#include <memory>
#include <limits>
#include <list>
#include "planner_common.h"
#include "container.h"

struct TreeNode{
    double* value = nullptr;
    int num_child = 0;  // for debugging purpose
    TreeNode * volatile firstChild = nullptr;
    TreeNode * volatile nextSibling = nullptr;
    TreeNode * volatile parent = nullptr;
    std::atomic_flag rewiring = ATOMIC_FLAG_INIT;  // only allow one node's child being rewired
    double cost_so_far = std::numeric_limits<double>::infinity();
    double cost_to_parent = std::numeric_limits<double>::infinity();
    double cost_to_goal = std::numeric_limits<double>::infinity();  // if infinity, do not connect it to goal, if infeasible

    void init(double *val);

    void add_child(TreeNode *node, double dist); 

    bool remove_child(TreeNode *node);

    void child_traversal(std::list<TreeNode*> &nodes);

    void child_traversal(List<TreeNode*> *nodes);
    
    void set_cost_recursive(double new_cost);

    void update_cost_old(double new_cost);

    void update_cost(double cost, TreeNode *goal);
};



typedef TreeNode RRT_Node;


// define the memory manager
// with a given allocation size, it allocates enough memory to be later operated on
class memory_manager{
public:
    memory_manager(int size_, int dim_) : size(size_), dim(dim_), inner_count(0) {
        data_region = (double*)malloc(size * dim * sizeof(double));
        node_region = malloc(size * sizeof(TreeNode));
    }

    std::pair<double*, TreeNode*> get_data_node() {
        int cur_count = inner_count;
        inner_count += 1;
        return std::make_pair(data_region + cur_count * dim, (TreeNode*)node_region + cur_count);
    }

    void reset() {
        inner_count = 0;
    }

    ~memory_manager() {
        free(node_region);
        free(data_region);
    }

private:
    int size, dim;  // size of nodes and dimension of data
    double *data_region;
    void *node_region;
    int inner_count;
};
#endif