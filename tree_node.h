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

void TreeNode::init(double *val) {
    value = val;
    num_child = 0;
    firstChild = nullptr;
    nextSibling = nullptr;
    parent = nullptr;
    rewiring.clear(std::memory_order_release);  // only allow one node's child being rewired
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
    while(rewiring.test_and_set(std::memory_order_acquire))
        ;
    __sync_synchronize();
    TreeNode *test = firstChild;
    if(test == node) {
        firstChild = test->nextSibling;  // maybe null and it is fine
        rewiring.clear(std::memory_order_release);
        return true;
    }
    TreeNode *prev_node;
    while(test != node) {
        if(test == nullptr){
            std::cout << "cannot find node\n";
            rewiring.clear(std::memory_order_release);
            return false;  // data may be corrupted, cannot find node within children lists
        }
        prev_node = test;
        test = test->nextSibling;
    }
    prev_node->nextSibling = test->nextSibling;  // this effectively removes node
    __sync_synchronize();
    rewiring.clear(std::memory_order_release);
    return true;
}

void TreeNode::child_traversal(std::list<TreeNode*> &nodes) {
    while(rewiring.test_and_set(std::memory_order_acquire))
        ;
    __sync_synchronize();
    TreeNode *to_insert = firstChild;
    while(to_insert != nullptr) {
        nodes.push_back(to_insert);
        to_insert = to_insert->nextSibling;
    }
    __sync_synchronize();
    rewiring.clear(std::memory_order_release);
}

void TreeNode::child_traversal(List<TreeNode*> *nodes) {
    while(rewiring.test_and_set(std::memory_order_acquire))
        ;
    __sync_synchronize();
    TreeNode *to_insert = firstChild;
    TreeNode **tmp;
    while(to_insert != nullptr) {
        std::tie(tmp, nodes) = nodes->get_one();
        *tmp = to_insert;
        to_insert = to_insert->nextSibling;
    }
    __sync_synchronize();
    rewiring.clear(std::memory_order_release);
}

// reduce cost for the subtree starting from this node
void TreeNode::set_cost_recursive(double new_cost) {
    while(rewiring.test_and_set(std::memory_order_acquire));
    __sync_synchronize();
    cost_so_far = new_cost;
    TreeNode *child = firstChild;
    while(child != nullptr) {
        double cost = cost_so_far + child->cost_to_parent;
        if(child->parent == this)
            child->set_cost_recursive(cost);
        else{
            std::cout << "child parent is not me\n";
        }
        child = child->nextSibling;
    }
    __sync_synchronize();
    rewiring.clear(std::memory_order_release);
}

void TreeNode::update_cost_old(double cost) {
    cost_so_far = cost;
    std::list<TreeNode*> nodes_list;
    nodes_list.push_back(this);
    child_traversal(nodes_list);
    nodes_list.push_back(nullptr);
    auto pit = nodes_list.begin(), nit = next(pit);
    while(nit != nodes_list.end()) {
        if(*nit == nullptr) {
            nit = nodes_list.erase(nit);
            pit = nodes_list.erase(pit);
            continue;
        }
        // make sure its parent is still pit
        TreeNode *parent = *pit;
        TreeNode *cur_node = *nit;
        if(cur_node->parent == parent) {  // this node is rewired to other nodes
            cur_node->cost_so_far = parent->cost_so_far + cur_node->cost_to_parent;
            cur_node->child_traversal(nodes_list);
            nodes_list.push_back(nullptr);
            nit++;
        }
        else {
            //std::cout << "parent changed\n";
            nit = nodes_list.erase(nit);
        }
    }
}

void TreeNode::update_cost(double cost, TreeNode *goal) {
    cost_so_far = cost;
    List<TreeNode*> nodes_list(100);
    List<TreeNode*> *list_nodes = &nodes_list;
    TreeNode **tmp;
    std::tie(tmp, list_nodes) = list_nodes->get_one();
    *tmp = this;
    child_traversal(list_nodes);
    std::tie(tmp, list_nodes) = list_nodes->get_one();
    *tmp = nullptr;
    // prepare for some traversal
    ListIterator<TreeNode*> pit, nit;
    pit.init(list_nodes);
    nit.init(list_nodes);
    nit.next();
    TreeNode *star_nit, *star_pit;
    while(nit.is_valid()) {
        star_nit = nit.get_val();
        if(star_nit == nullptr) {
            nit.next();
            pit.next();
            continue;
        }
        // make sure its parent is still pit
        TreeNode *parent = pit.get_val();
        TreeNode *cur_node = nit.get_val();
        if(cur_node->parent == parent) {  // this node is rewired to other nodes
            cur_node->cost_so_far = parent->cost_so_far + cur_node->cost_to_parent;
            if((cur_node->cost_to_goal != std::numeric_limits<double>::infinity()) && (cur_node->cost_so_far + cur_node->cost_to_goal < goal->cost_so_far)) {
                //printf("re %f %f\n", cur_node->cost_so_far + cur_node->cost_to_goal, goal->cost_so_far);
                goal->cost_so_far = cur_node->cost_so_far + cur_node->cost_to_goal;
                goal->parent = cur_node;
                goal->cost_to_parent = cur_node->cost_to_goal;
            }
            cur_node->child_traversal(list_nodes);
            std::tie(tmp, list_nodes) = list_nodes->get_one();
            *tmp = nullptr;
            nit.next();
        }
        else {
            //std::cout << "parent changed\n";
            nit.next();
        }
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