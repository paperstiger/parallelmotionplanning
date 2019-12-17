/*
This file exposes several class and functions.
It tries to make the project more organized.
*/
#ifndef PRRTSTAR_H
#define PRRTSTAR_H
#include <atomic>
#include <mutex>
#include "kdtree.h"
#include "planner_common.h"
#include "tree_node.h"

class RRT{
public:
    double extend_radius;
    gVector start, goal;
    kd_tree_t *tree;
    std::vector<Env*> *envs;
    Env *env;
    RRT_Node *goal_node, *start_node;
    RRTOption *options;
    std::atomic<int> tree_size;
    std::vector<memory_manager*> managers;
    std::mutex m;
    bool resample_to_feasible;

    RRT() : tree(nullptr), env(nullptr), goal_node(nullptr) {
        extend_radius = 0.1;
        resample_to_feasible = true;
        options = new RRTOption;
    }

    void set_start_goal(const gVector &start_, const gVector &goal_) {
        start = start_;
        goal = goal_;
    }

    void set_envs(std::vector<Env*> *_envs) {
        envs = _envs;
        env = _envs->at(0);
    }

    // following Kris convention, this function samples more
    bool plan_more(int num, int thread_num);

    // return the path
    std::vector<gVector> get_path();

    double get_path_length() const {
        return goal_node->cost_so_far;
    }

    ~RRT() {
        kd_free(tree);
        delete options;
        delete start_node;
        delete goal_node;
        for(auto &mem : managers)
            delete mem;
    }

    void update_goal_parent(RRT_Node *node, double radius, Env *_env);

    void _extend_within(double *x, const double *base, double factor) const {
        for(size_t i = 0; i < env->dim; i++) {
            x[i] = base[i] + factor * (x[i] - base[i]);
        }
    }
};

#endif