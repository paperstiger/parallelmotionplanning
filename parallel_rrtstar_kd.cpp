/*
Write parallel version of RRT star, just make everything work.
We have to create threads on our own and maintain the RRT tree when multiple threads are adding children
*/
#include <vector>
#include <list>
#include <iostream>
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
    std::atomic<TreeNode*> firstChild, lastChild, nextSibling;
    double cost_so_far = std::numeric_limits<double>::infinity();
    double cost_to_parent = std::numeric_limits<double>::infinity();
    TreeNode *parent;
    TreeNode(const T &val) : value(val),
                             firstChild(nullptr),
                             lastChild(nullptr),
                             nextSibling(nullptr),
                             parent(nullptr)
                             {}
    void add_child(TreeNode<T> *node) {
        while(true){
            TreeNode *first = firstChild.load(std::memory_order_relaxed),
                    *last = lastChild.load(std::memory_order_relaxed),
                    *next = nextSibling.load(std::memory_order_relaxed);
            if(!first) {  // no child at all, we modify several stuff
                if(firstChild.compare_exchange_weak(first, node, std::memory_order_release, std::memory_order_relaxed)) {
                    if(lastChild.compare_exchange_weak(last, node, std::memory_order_release, std::memory_order_relaxed))
                        break;
                }
            }
            else{  // last child has to assign to it, lastChild's sibling has to set to it, also set firstChild's sibling
                if(lastChild.compare_exchange_weak(last, node, std::memory_order_release, std::memory_order_relaxed)){
                    last->nextSibling = node;  // I bet it is done correctly, last is set to node
                    break;
                }
            }
        }
        node->parent = this;
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
void thread_fun(RRT *problem, int sample_num);

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
            goal_node = new RRT_Node(goal);
            kd_insert(tree, goal_node->value.data(), goal_node);  // RRT star has to start with two nodes
            tree_size.store(2, std::memory_order_relaxed);
        }
        std::vector<std::thread> threads;
        for(int i = 0; i < thread_num; i++) {
            threads.push_back(std::thread(thread_fun, this, num));
        }
        for(auto &t : threads)
            t.join();
        return true;  // no exception raised means success
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

void thread_fun(RRT *problem, int sample_num)
{
    //std::cout << "Entering thread " << std::this_thread::get_id() << std::endl;
    list_pnter_dist lst_pnt_dis;
    for(int i = 0; i < sample_num; i++) {
        Vector x = problem->env->sample();
        // find closest point
        double dist = 0;
        int step_no = problem->tree_size.load();
        double radius = problem->options->gamma *  // the radius being queried
                pow(log((double)step_no + 1.0) / (double)(step_no + 1.0),
                    1.0 / (double)problem->env->dim);
        int near_list_size = kd_near(problem->tree, x.data(), radius, kd_near_call_back, &lst_pnt_dis);
        // in two conditions, we switch to nearest neighbor search
        // 1. no node found in the vinicity
        // 2. one node is found but is the goal node; the goal node shall not have any children
        if((near_list_size == 0) || ((near_list_size == 1) && (lst_pnt_dis.front().first == problem->goal_node)) {
            double nearest_dist = 0;
            RRT_Node *near_node = (RRT_Node*)kd_nearest(problem->tree, x.data(), &nearest_dist);
            if(near_node == problem->goal_node)  // well, my nn is the goal, let me reject this one
                continue;
            // extend x so that its distance is exact radius
            problem->_extend_within(x, close_node->value, radius / nearest_dist);
            // check collision
            if(!problem->env->is_free(x, near_node->value))  // sample is infeasible from near_node
                continue;
            // create node and return
            RRT_Node *new_node = new RRT_Node(x);
            new_node->cost_to_parent = radius;
            new_node->cost_so_far = near_node->cost_so_far + radius;  // since the distance is apparently radius
            near_node->add_child(new_node);
            // no need to update best path unless new_node is close to goal, we only have to check goal
            // TODO: it may be useful to check if new_node is close to target...
            kd_insert(problem->tree, new_node->value.data(), new_node);
        }
        else{  // link new node to the nearest one and perform rewiring based on that
            // first find the closest node to connect into
            sort(lst_pnt_dis.begin(), lst_pnt_dis.end(), [](const pvd &n1, const pvd &n2) {return n1.second < n2.second;});
            for(auto it = lst_pnt_dis.begin(); it != lst_pnt_dis.end(); it++) {
                RRT_Node *cand = (RRT_Node*)(it->first);
                if(cand == problem->goal_node)  // this is the goal node, skip it
                    continue;
                if(problem->env->is_free(x.data(), cand->value.data())) {  //  collision free, can add node
                    RRT_Node *new_node = new RRT_Node(x);
                    new_node->cost_to_parent = it->second;
                    new_node->cost_so_far = cand->cost_so_far + it->second;
                    cand->add_child(new_node);
                    kd_insert(problem->tree, new_node->value.data(), new_node);
                    // try to rewire
                    for (auto rit=lst_pnt_dis.rbegin(); rit != lst_pnt_dis.rend(); rit++) {
                        // for those
                                rewire(worker,
                                       worker->near_list[j].link,
                                       worker->near_list[j].link_cost,
                                       new_node,
                                       radius);

                                /* TODO: reference release on near_list[j].link */
                        }
                }
            }
        }
        // find all points near by
        RRT_Node *close_node = (RRT_Node*)kd_nearest(problem->tree, x.data(), &dist);
        // std::cout << "knn dist = " << dist << std::endl;
        if(dist > problem->extend_radius)
            problem->_extend_within(x, close_node->value, dist);
        // check if collision free
        if(problem->env->is_free(x, close_node->value)){
            auto prev_node = close_node;
            RRT_Node *new_node = new RRT_Node(x);
            prev_node->add_child(new_node);
            kd_insert(problem->tree, new_node->value.data(), new_node);
            problem->tree_size.fetch_add(1, std::memory_order_relaxed);
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
    std::cout << "Running with " << thread_num << "threads\n";
    srand(time(0));
    NaiveEnv env;
    RRT rrt(&env);
    rrt.extend_radius = 0.01;
    Vector start = {0, 0}, goal = {1, 1};
    rrt.set_start_goal(start, goal);
    auto tnow = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < 1; i++) {
        std::cout << "i = " << i << std::endl;
        if(rrt.plan_more(100000 / thread_num, thread_num))
            break;
    }
    auto tf = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(tf - tnow);
    std::cout << "Time taken by function: "
         << duration.count() << " microseconds" << std::endl;
    auto path = rrt.get_path();
    std::cout << "path length " << path.size() << std::endl;
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
