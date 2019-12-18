#ifndef TREE_NODE_H
#define TREE_NODE_H
#include <iostream>
#include <memory>
#include <limits>
#include <list>
#include <tuple>
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
        data_regions.push_back(data_region);
        node_regions.push_back(node_region);
        inner_count = 0;
        _last_piece_inner_count = 0;
        _iter_mode = false;
        _iter_vec_idx = 0;
    }

    std::pair<double*, TreeNode*> get_data_node() {
        if(!_iter_mode){
            if(inner_count < size){
                int cur_count = inner_count;
                inner_count += 1;
                _last_piece_inner_count = inner_count;
                return std::make_pair(data_region + cur_count * dim, (TreeNode*)node_region + cur_count);
            }
            else{
                // we have to allocate new memory
                data_region = (double*)malloc(size * dim * sizeof(double));
                node_region = malloc(size * sizeof(TreeNode));
                data_regions.push_back(data_region);
                node_regions.push_back(node_region);
                inner_count = 0;
                return get_data_node();
            }
        }
        else{
            if(inner_count < size) {
                int cur_count = inner_count;
                inner_count++;
                return std::make_pair(data_region + cur_count * dim, (TreeNode*)node_region + cur_count);
            }
            else{
                _iter_vec_idx++;
                inner_count = 0;
                data_region = data_regions[_iter_vec_idx];
                node_region = node_regions[_iter_vec_idx];
                return get_data_node();
            }
        }
    }

    void reset() {
        if(!_iter_mode) {  // we want to switch to iter mode
            inner_count = 0;
            _iter_vec_idx = 0;
            data_region = data_regions[0];
            node_region = node_regions[0];
        }
        else {
            inner_count = _last_piece_inner_count;
            data_region = data_regions.back();
            node_region = node_regions.back();
        }
        _iter_mode = !_iter_mode;
    }

    int get_size() {
        return _last_piece_inner_count + size * (data_regions.size() - 1);
    }

    ~memory_manager() {
        for(auto node_r : node_regions)
            free(node_r);
        for(auto data_r : data_regions)
            free(data_r);
    }

private:
    int size, dim;  // size of nodes and dimension of data
    std::vector<double*> data_regions;
    std::vector<void*> node_regions;
    double *data_region;
    void *node_region;
    int inner_count;
    int _last_piece_inner_count;
    bool _iter_mode;
    int _iter_vec_idx;
};
#endif