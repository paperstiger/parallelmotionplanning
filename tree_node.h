#ifndef TREE_NODE_H
#define TREE_NODE_H
#include <iostream>
#include <memory>
#include <limits>
#include "planner_common.h"

struct TreeNode{
    double* value = nullptr;
    int num_child = 0;  // for debugging purpuse
    TreeNode* firstChild = nullptr;
    TreeNode* nextSibling = nullptr;
    TreeNode* parent = nullptr;
    bool rewired = false;  // only allow one node's child being rewired
    double cost_so_far = std::numeric_limits<double>::infinity();
    double cost_to_parent = std::numeric_limits<double>::infinity();

    void init(double *val);

    void add_child(TreeNode *node, double dist); 

    bool remove_child(TreeNode *node);
    
    void set_cost_recursive(double new_cost);
};

void TreeNode::init(double *val) {
    value = val;
    num_child = 0;
    firstChild = nullptr;
    nextSibling = nullptr;
    parent = nullptr;
    rewired = false;  // only allow one node's child being rewired
    cost_so_far = std::numeric_limits<double>::infinity();
    cost_to_parent = std::numeric_limits<double>::infinity();
}

void TreeNode::add_child(TreeNode *node, double dist){
    TreeNode *first = nullptr;
    do {
        first = firstChild;
        node->nextSibling = first;
    }while(!__sync_bool_compare_and_swap(&firstChild, first, node));
    node->parent = this;
    node->cost_to_parent = dist;
    node->cost_so_far = cost_so_far + dist;
    num_child += 1;
}

bool TreeNode::remove_child(TreeNode *node) {
    TreeNode *test = firstChild;
    if(test == node) {
        firstChild = test->nextSibling;  // maybe null and it is fine
        return true;
    }
    TreeNode *prev_node;
    while(test != node) {
        if(test == nullptr){
            std::cout << "cannot find node\n";
            return false;  // data may be corrupted, cannot find node within children lists
        }
        prev_node = test;
        test = test->nextSibling;
    }
    prev_node->nextSibling = test->nextSibling;  // this effectively removes node
    return true;
}

// reduce cost for the subtree starting from this node
void TreeNode::set_cost_recursive(double new_cost) {
    cost_so_far = new_cost;
    TreeNode *child = firstChild;
    while(child != nullptr) {
        double cost = cost_so_far + child->cost_to_parent;
        child->set_cost_recursive(cost);
        child = child->nextSibling;
    }
}

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