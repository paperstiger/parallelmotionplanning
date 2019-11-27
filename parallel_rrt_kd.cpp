/*
Write parallel version of RRT, just make everything work.
We have to create threads on our own and maintain the RRT tree when multiple threads are adding children
*/
#include <vector>
#include <list>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
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

// this class maintains some cool stuff
template<class T>
class TreeNode{
public:
    T value;
    std::atomic<TreeNode*> firstChild, lastChild, nextSibling;
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

class RRT;
// write subroutine for each thread
// problem is a pointer to the RRT instance
// found is a global indicator of exit, does not have to be atomic
// sample_num is per-thread sampling numbers
void thread_fun(RRT *problem, bool &found, int sample_num);

class RRT{
public:
    double extend_radius;
    Vector start, goal;
    kd_tree_t *tree;
    Env *env;
    RRT_Node *_goal_node;
    std::mutex _mutex;

    void set_start_goal(const Vector &start_, const Vector &goal_) {
        start = start_;
        goal = goal_;
    }

    RRT(Env *env_) : tree(nullptr), env(env_), _goal_node(nullptr) {
        extend_radius = 0.1;
    }

    // following Kris convention, this function samples more
    bool plan_more(int num, int thread_num) {
        if(!tree){
            RRT_Node *start_node = new RRT_Node(start);
            tree = kd_create_tree(env->dim, env->min.data(), env->max.data(),
                        MyDistFun,
                        start.data(), start_node);
        }
        std::vector<std::thread> threads;
        bool found = false;
        for(int i = 0; i < thread_num; i++) {
            threads.push_back(std::thread(thread_fun, this, std::ref(found), num));
        }
        for(auto &t : threads)
            t.join();
        return found;
    }

    // return the path
    std::vector<Vector> get_path() {
        std::vector<Vector> path;
        if(!_goal_node)
            return path;
        std::list<Vector> list_q;
        auto node = _goal_node;
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

    void _extend_within(Vector &x, const Vector &base, double dist) const {
        double factor = extend_radius / dist;
        const Vector &pnt = base;
        for(size_t i = 0; i < x.size(); i++) {
            x[i] = pnt[i] + factor * (x[i] - pnt[i]);
        }
    }
};

void thread_fun(RRT *problem, bool &found, int sample_num)
{
    //std::cout << "Entering thread " << std::this_thread::get_id() << std::endl;
    for(int i = 0; i < sample_num; i++) {
        if(found)
            break;
        Vector x = problem->env->sample();
        // find closest point
        double dist = 0;
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
            // check if this node reaches goal
            if ((ArrayUtils::Distance_L2(new_node->value, problem->goal) < problem->extend_radius) &&
                (problem->env->is_free(new_node->value, problem->goal)))
            {
                std::lock_guard<std::mutex>(problem->_mutex);
                // eventually we can add goal node
                if(!found) {
                    problem->_goal_node = new RRT_Node(problem->goal);
                    new_node->add_child(problem->_goal_node);
                    found = true;
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
