/*
Write parallel version of RRT star, just make everything work.
We have to create threads on our own and maintain the RRT tree when multiple threads are adding children
*/
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <tuple>
#include <limits>
#include <algorithm>
#include "stdlib.h"
#include "kdtree.h"
#include "arrayutils.h"

typedef std::vector<double> Vector;

// a base class for collision checker, it should be implemented by an environment
class CollisionChecker{
public:
    virtual bool is_free(const Vector &v1, const Vector &v2) = 0;
};

// a base class for an environment, it has to have to implement some functions
class Env : public CollisionChecker{
public:
    virtual Vector sample() = 0;

    Env(size_t dim_) : dim(dim_) {} 

    size_t dim;
    Vector min, max;
};

extern "C" double MyDistFun(const double *a, const double *b)
{
    double dist = 0;
    for(int i = 0; i < 2; i++)
        dist += pow(a[i] - b[i], 2);
    return sqrt(dist);
}

typedef std::pair<void*, double> pvd;
typedef std::list<pvd> list_pnter_dist;
// data is the writing location, value is the stored value for each kd node
void kd_near_call_back(void *data, int no, void *value, double dist) {
    list_pnter_dist *lst = (list_pnter_dist*)data;
    if(no == 0)
        lst->clear();
    lst->push_back(std::make_pair(value, dist));
}

// this class maintains some cool stuff
template<class T>
class TreeNode{
public:
    T value;
    int num_child = 0;  // for debugging purpuse
    std::atomic<TreeNode*> firstChild, nextSibling;
    TreeNode *parent = nullptr;
    std::atomic<bool> rewired;  // only allow one node's child being rewired
    double cost_so_far = std::numeric_limits<double>::infinity();
    double cost_to_parent = std::numeric_limits<double>::infinity();
    TreeNode(const T &val) : value(val)
                             {rewired = false;
                             firstChild = nullptr;
                             nextSibling = nullptr;}
    void add_child(TreeNode<T> *node, double dist) {
        TreeNode *first;
        do {
            first = firstChild.load(std::memory_order_relaxed);
            node->nextSibling = first;
        }while(!firstChild.compare_exchange_weak(first, node, std::memory_order_release, std::memory_order_relaxed));
        node->parent = this;
        node->cost_to_parent = dist;
        node->cost_so_far = cost_so_far + dist;
        num_child += 1;
    }

    bool remove_child(TreeNode<T> *node) {
        TreeNode *test = firstChild.load(std::memory_order_relaxed);
        if(test == node) {
            firstChild.store(test->nextSibling);  // maybe null and it is fine
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
        prev_node->nextSibling.store(test->nextSibling);  // this effectively removes node
        return true;
    }

    // reduce cost for the subtree starting from this node
    void set_cost_recursive(double new_cost) {
        cost_so_far = new_cost;
        TreeNode *child = firstChild.load();
        while(child) {
            double cost = cost_so_far + child->cost_to_parent;
            child->set_cost_recursive(cost);
            child = child->nextSibling.load();
        }
    }
};

typedef TreeNode<Vector> RRT_Node;

struct RRTOption {
    double gamma = 1;
};
class RRT;
// write subroutine for each thread
// problem is a pointer to the RRT instance
// found is a global indicator of exit, does not have to be atomic
// sample_num is per-thread sampling numbers
void thread_fun(RRT *problem, int sample_num, int);

class RRT{
public:
    double extend_radius;
    Vector start, goal;
    kd_tree_t *tree;
    Env *env;
    RRT_Node *goal_node;
    RRTOption *options;
    std::atomic<int> tree_size;

    void set_start_goal(const Vector &start_, const Vector &goal_) {
        start = start_;
        goal = goal_;
    }

    RRT(Env *env_) : tree(nullptr), env(env_), goal_node(nullptr) {
        extend_radius = 0.1;
        options = new RRTOption;
    }

    // following Kris convention, this function samples more
    bool plan_more(int num, int thread_num) {
        if(!tree){
            RRT_Node *start_node = new RRT_Node(start);
            start_node->cost_so_far = 0;  // I do not modify cost_to_parent since it has no parents
            tree = kd_create_tree(env->dim, env->min.data(), env->max.data(),
                        MyDistFun,
                        start.data(), start_node);
            goal_node = new RRT_Node(goal);  // goal node is created but not in in kd_tree
            // kd_insert(tree, goal_node->value.data(), goal_node);  // RRT star has to start with two nodes
            tree_size.store(1, std::memory_order_relaxed);
        }
        std::vector<std::thread> threads;
        for(int i = 0; i < thread_num; i++) {
            threads.push_back(std::thread(thread_fun, this, num, i));
        }
        for(auto &t : threads)
            t.join();
        return true;  // no exception raised means success
    }

    void update_goal_parent(RRT_Node *node, double radius) {
        double dist = MyDistFun(node->value.data(), goal.data());
        if((dist > radius) || (!env->is_free(node->value, goal)))
            return;
        if(node->cost_so_far + dist < goal_node->cost_so_far) {
            if(goal_node->parent)
                goal_node->parent->remove_child(goal_node);
            node->add_child(goal_node, dist);
        }
    }

    // return the path
    std::vector<Vector> get_path() {
        std::vector<Vector> path;
        if(goal_node->cost_so_far == std::numeric_limits<double>::infinity())
            return path;
        std::list<Vector> list_q;
        auto node = goal_node;
        while(node) {
            list_q.push_front(node->value);
            node = node->parent;
        }
        for(auto iter=list_q.begin(); iter != list_q.end(); iter++)
            path.push_back(*iter);
        return path;
    }

    double get_path_length() const {
        return goal_node->cost_so_far;
    }

    ~RRT() {
        delete tree;
    }

    void _extend_within(Vector &x, const Vector &base, double factor) const {
        const Vector &pnt = base;
        for(size_t i = 0; i < x.size(); i++) {
            x[i] = pnt[i] + factor * (x[i] - pnt[i]);
        }
    }
};

void thread_fun(RRT *problem, int sample_num, int thread_id)
{
    //std::cout << "Entering thread " << std::this_thread::get_id() << std::endl;
    list_pnter_dist lst_pnt_dis;
    double goal_radius = 0.1;
    for(int i = 0; i < sample_num; i++) {
        //std::cout << " i = " << i << std::endl;
        Vector x = problem->env->sample();
        // find closest point
        double dist = 0;
        int step_no = problem->tree_size.load();
        double radius = problem->options->gamma *  // the radius being queried
                pow(log((double)step_no + 1.0) / (double)(step_no + 1.0),
                    1.0 / (double)problem->env->dim);
        //std::cout << "radius = " << radius << "\n";
        lst_pnt_dis.clear();
        int near_list_size = kd_near(problem->tree, x.data(), radius, kd_near_call_back, &lst_pnt_dis);
        // in two conditions, we switch to nearest neighbor search
        // 1. no node found in the vinicity
        // 2. one node is found but is the goal node; the goal node shall not have any children
        if((near_list_size == 0)) {
            double nearest_dist = 0;
            RRT_Node *near_node = (RRT_Node*)kd_nearest(problem->tree, x.data(), &nearest_dist);
            // extend x so that its distance is exact radius
            problem->_extend_within(x, near_node->value, radius / nearest_dist);
            // check collision
            if(!problem->env->is_free(x, near_node->value))  // sample is infeasible from near_node
                continue;
            // create node and return, no rewiring is needed
            RRT_Node *new_node = new RRT_Node(x);
            near_node->add_child(new_node, radius);
            // no need to update best path unless new_node is close to goal, we only have to check goal
            // TODO: it may be useful to check if new_node is close to target...
            kd_insert(problem->tree, new_node->value.data(), new_node);
            problem->tree_size.fetch_add(1, std::memory_order_relaxed);
            // check if goal can be routed here
            problem->update_goal_parent(new_node, goal_radius);
        }
        else{  // link new node to the nearest one and perform rewiring based on that
            // first find the closest node to connect into
            lst_pnt_dis.sort([](const pvd &n1, const pvd &n2) {return n1.second < n2.second;});
            //std::cout << "rewire " << lst_pnt_dis.size() << " nodes\n";
            for(auto it = lst_pnt_dis.begin(); it != lst_pnt_dis.end(); it++) {
                RRT_Node *cand = (RRT_Node*)(it->first);
                if(cand == problem->goal_node)  // this is the goal node, we rewire based on if its has parent
                    continue;
                if(problem->env->is_free(x, cand->value)) {  //  collision free, can add node
                    RRT_Node *new_node = new RRT_Node(x);
                    cand->add_child(new_node, it->second);
                    problem->update_goal_parent(new_node, goal_radius);
                    kd_insert(problem->tree, new_node->value.data(), new_node);
                    // try to rewire
                    for (auto rit=lst_pnt_dis.rbegin(); rit != lst_pnt_dis.rend(); rit++) {
                        RRT_Node *cani = (RRT_Node*)(rit->first);
                        double cani_dist = rit->second;
                        // rewire only if gets a shorter path
                        if(cani_dist + new_node->cost_so_far >= cani->cost_so_far) {
                            continue;
                        }
                        // collision free has to be satisfied
                        if(!problem->env->is_free(x, cani->value))
                            continue;
                        // rewiring is needed
                        // we need to delete this node, rewire to another node
                        // traversal needs to make sure prev->next is itself
                        // delete needs to make sure prev and next are the same
                        // first, search for cani in cani's parent's children
                        RRT_Node *parent = cani->parent;
                        bool rmflag = parent->remove_child(cani);
                        if(!rmflag)
                            std::cout << "at thread " << thread_id << " iteration " << i << "\n";
                        new_node->add_child(cani, cani_dist);
                        // hopefully we can play with cani now
                        double new_cost = cani->cost_so_far;
                        cani->set_cost_recursive(new_cost);
                    }
                    break;
                }
            }
        }
    }
}

// a naive environment simply from (0, 0) to (1, 1), no obstacle
class NaiveEnv : public Env {
public:
    NaiveEnv() : Env(2) {
        min = {0, 0};
        max = {1, 1};
    }

    bool is_free(const Vector &v1, const Vector &v2) {
        double x1 = v1[0], x2 = v2[0], y1 = v1[1], y2 = v2[1];
        if((x1 - 0.5) * (x2 - 0.5) >= 0)
            return true;
        else{
            double factor = (0.5 - x2) / (x1 - x2);
            double interpy = y2 + factor * (y1 - y2);
            return (interpy > 0.8) | (interpy < 0.2);
        }
    }

    Vector sample() {
        Vector p(dim);
        for(int i = 0; i < dim; i++) {
            p[i] = ((double)rand() / RAND_MAX * (max[i] - min[i]) + min[i]);
        }
        return p;
    }
};

int main(int argc, char *argv[]) {
    int thread_num = 1;
    if(argc > 1) {
        thread_num = std::atoi(argv[1]);
    }
    std::cout << "Running with " << thread_num << " threads\n";
    srand(time(0));
    NaiveEnv env;
    RRT rrt(&env);
    rrt.options->gamma = 0.2;
    rrt.extend_radius = 0.01;
    Vector start = {0, 0}, goal = {1, 1};
    rrt.set_start_goal(start, goal);
    auto tnow = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < 1; i++) {
        std::cout << "i = " << i << std::endl;
        if(rrt.plan_more(10000 / thread_num, thread_num))
            break;
    }
    auto tf = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(tf - tnow);
    std::cout << "Time taken by function: "
         << (double)duration.count() / 1000000 << " seconds" << std::endl;
    auto path = rrt.get_path();
    std::ofstream myfile("path.txt", std::ios::out);
    for(auto &p : path) {
        for(auto &n : p)
            myfile << n << " ";
        myfile << std::endl;
    }
    myfile.close();
    std::cout << "path size " << path.size() << " length " << rrt.get_path_length() << std::endl;
    // int stone = 0;
    // for(auto &p : path) {
    //     std::cout << stone << " ";
    //     for(auto &n : p)
    //         std::cout << n << " ";
    //     std::cout << "\n";
    //     stone++;
    // }
    return 0;
}
