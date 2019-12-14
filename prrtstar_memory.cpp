/*
I do not know what is going on, but is memory allocation takes too long?
I also want to test if I can manage memory on my own.
*/
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <tuple>
#include <limits>
#include <algorithm>
#include <cstring>
#include "stdlib.h"
#include "kdtree.h"
#include "arrayutils.h"

#include "planner_common.h"
#include "tree_node.h"

//
#include <math.h>
//
double MyDistFun(const double *a, const double *b)
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
class RRT;
// write subroutine for each thread
// problem is a pointer to the RRT instance
// found is a global indicator of exit, does not have to be atomic
// sample_num is per-thread sampling numbers
void thread_fun(RRT *problem, int sample_num, int);
bool check_node_correctness(RRT_Node *node);
bool check_tree_correctness(RRT_Node *start_node);
std::condition_variable cv;
int to_processed;
std::atomic<int> finished;
const bool TEST_TREE_VALIDITY = false;


class RRT{
public:
    double extend_radius;
    Vector start, goal;
    kd_tree_t *tree;
    Env *env;
    RRT_Node *goal_node, *start_node;
    RRTOption *options;
    std::atomic<int> tree_size;
    std::vector<memory_manager*> managers;
    std::mutex m;

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
            start_node = new RRT_Node;
            start_node->value = start.data();
            start_node->cost_so_far = 0;  // I do not modify cost_to_parent since it has no parents
            tree = kd_create_tree(env->dim, env->min.data(), env->max.data(),
                        MyDistFun,
                        start.data(), start_node);
            goal_node = new RRT_Node;  // goal node is created but not in in kd_tree
            goal_node->value = goal.data();
            // kd_insert(tree, goal_node->value.data(), goal_node);  // RRT star has to start with two nodes
            tree_size.store(1, std::memory_order_relaxed);
        }
        std::vector<std::thread> threads;
        managers.resize(thread_num);
        to_processed = thread_num;
        finished.store(0);
        for(int i = 0; i < thread_num; i++) {
            threads.push_back(std::thread(thread_fun, this, num, i));
        }
        for(auto &t : threads)
            t.join();
        return true;  // no exception raised means success
    }

    void update_goal_parent(RRT_Node *node, double radius) {
        double dist = MyDistFun(node->value, goal.data());
        if((dist > radius) || (!env->is_free(node->value, goal.data())))
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
            list_q.push_front(Vector(node->value, node->value + env->dim));
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
        kd_free(tree);
        delete options;
        delete start_node;
        delete goal_node;
        for(auto &mem : managers)
            delete mem;
    }

    void _extend_within(double *x, const double *base, double factor) const {
        for(size_t i = 0; i < env->dim; i++) {
            x[i] = base[i] + factor * (x[i] - base[i]);
        }
    }
};

void thread_fun(RRT *problem, int sample_num, int thread_id)
{
    //std::cout << "Entering thread " << std::this_thread::get_id() << std::endl;
    memory_manager *mem_pnter = new memory_manager(sample_num, problem->env->dim);
    problem->managers[thread_id] = mem_pnter;
    int thread_num = problem->managers.size();
    auto &memory = *mem_pnter;
    list_pnter_dist nn_list;
    double goal_radius = 0.1;
    double *cmndata;
    RRT_Node *cmnnode;
    double *x = new double[problem->env->dim];
    int DIM = problem->env->dim;
    for(int i = 0; i < sample_num; i++) {
        problem->env->sample(x, thread_id, thread_num);
        // find closest point
        double dist = 0;
        int step_no = problem->tree_size.load();
        double radius = problem->options->gamma *  // the radius being queried
                pow(log((double)step_no + 1.0) / (double)(step_no + 1.0),
                    1.0 / (double)problem->env->dim);
        //std::cout << "radius = " << radius << "\n";
        nn_list.clear();
        int near_list_size = kd_near(problem->tree, x, radius, kd_near_call_back, &nn_list);
        // in two conditions, we switch to nearest neighbor search
        // 1. no node found in the vinicity
        // 2. one node is found but is the goal node; the goal node shall not have any children
        bool new_node_created = false;
        if((near_list_size == 0)) {
            double nearest_dist = 0;
            RRT_Node *near_node = (RRT_Node*)kd_nearest(problem->tree, x, &nearest_dist);
            // extend x so that its distance is exact radius
            problem->_extend_within(x, near_node->value, radius / nearest_dist);
            // check collision
            if(!problem->env->is_free(x, near_node->value)) {  // sample is infeasible from near_node
                i--;  // make sure enough feasible samples are collected
                continue;
            }
            // create node and return, no rewiring is needed
            std::tie(cmndata, cmnnode) = memory.get_data_node();
            std::memcpy(cmndata, x, DIM * sizeof(double));
            cmnnode->init(cmndata);
            RRT_Node *new_node = cmnnode;
            near_node->add_child(new_node, radius);
            // no need to update best path unless new_node is close to goal, we only have to check goal
            // TODO: it may be useful to check if new_node is close to target...
            kd_insert(problem->tree, new_node->value, new_node);
            problem->tree_size.fetch_add(1, std::memory_order_relaxed);
            // check if goal can be routed here
            problem->update_goal_parent(new_node, goal_radius);
        }
        else{  // link new node to the nearest one and perform rewiring based on that
            // first find the closest node to connect into
            //lst_pnt_dis.sort([](const pvd &n1, const pvd &n2) {return n1.second < n2.second;});
            //std::cout << "rewire " << lst_pnt_dis.size() << " nodes\n";
            for(auto it = nn_list.begin(); it != nn_list.end(); it++) {
                RRT_Node *cand = (RRT_Node*)(it->first);
                if(cand == problem->goal_node)  // this is the goal node, we rewire based on if its has parent
                    continue;
                if(problem->env->is_free(x, cand->value)) {  //  collision free, can add node
                    std::tie(cmndata, cmnnode) = memory.get_data_node();
                    new_node_created = true;
                    std::memcpy(cmndata, x, DIM * sizeof(double));
                    cmnnode->init(cmndata);
                    RRT_Node *new_node = cmnnode;
                    cand->add_child(new_node, it->second);
                    problem->update_goal_parent(new_node, goal_radius);
                    kd_insert(problem->tree, new_node->value, new_node);
                    // try to rewire
                    for (auto rit = nn_list.rbegin(); rit != nn_list.rend(); rit++) {
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
                        if(!rmflag){
                            std::cout << "at thread " << thread_id << " iteration " << i << "\n";
                            continue;  // TODO: may remove cani from its new parents if possible
                        }
                        new_node->add_child(cani, cani_dist);
                        // hopefully we can play with cani now
                        double new_cost = cani->cost_so_far;
                        // cani->set_cost_recursive(new_cost);
                        cani->update_cost(new_cost);
                    }
                    break;
                }
            }
            if(!new_node_created)
                i--;
        }
    }
    delete[] x;
    finished.fetch_add(1);
    std::unique_lock<std::mutex> lk(problem->m);
    cv.wait(lk, []{return to_processed == finished;});
    //check if trees is valid
    if(TEST_TREE_VALIDITY) {
        if(thread_id == 0){
            if (check_tree_correctness(problem->start_node)){
                std::cout << "Tree Valid " << std::endl;
            }
            else{
                std::cout << "Tree InValid " << std::endl;
            }
        }
        //now check if there is any node "floating"
        memory.reset(); 
        double *tmpdata;
        RRT_Node *tmpnode;
        int counter = 0;
        for (int i = 0; i < sample_num ; i++){    
            std::tie(tmpdata, tmpnode)=memory.get_data_node();
            if (tmpnode->parent == nullptr) {
                counter++;
            }
            else if (fabs(tmpnode->parent->cost_so_far + tmpnode->cost_to_parent - tmpnode->cost_so_far) > 1e-10)
            {
                counter++;
            }
            
        }
        std::cout << "thread:"<< " " <<thread_id  << " " << "has " << counter << " " << "have parental issues" << std::endl;
    }
    tl_free(&problem->tree->mempool, NULL);
    lk.unlock();
    cv.notify_one();
}

// a naive environment simply from (0, 0) to (1, 1), no obstacle
class NaiveEnv : public Env {
public:
    NaiveEnv() : Env(2) {
        min = {0, 0};
        max = {1, 1};
    }

    bool is_free(const double *v1, const double *v2) {
        double x1 = v1[0], x2 = v2[0], y1 = v1[1], y2 = v2[1];
        if((x1 - 0.5) * (x2 - 0.5) >= 0)
            return true;
        else{
            double factor = (0.5 - x2) / (x1 - x2);
            double interpy = y2 + factor * (y1 - y2);
            return (interpy > 0.8) | (interpy < 0.2);
        }
    }

    void sample(double *x, int id, int num) {
        if(num <= 1) {  // num if not greater than 1
            double *p = x;
            for(int i = 0; i < dim; i++) {
                p[i] = ((double)rand() / RAND_MAX * (max[i] - min[i]) + min[i]);
            }
        }
        else{
            x[0] = (id + (double)rand() / RAND_MAX) / num * (max[0] - min[0]) + min[0];
            x[1] = ((double)rand() / RAND_MAX * (max[1] - min[1]) + min[1]);
        }
    }
};
bool check_tree_correctness(RRT_Node *start_node){
    RRT_Node *child_node = start_node->firstChild;
    while(child_node != nullptr){
        if(!check_node_correctness(child_node)){
            return false;
        }
        child_node = child_node->nextSibling;
    }
    return true;
}

bool check_node_correctness(RRT_Node *node){
    //auto start_node = problem->start_node;
    //std::cout <<  "test start node:" << start_node->cost_so_far << std::endl;
    double epsilon = 0.000001;
    // checking 1) if the cost_so_far equals the cost_so_far from parent + cost_to_parent
    // 2) if parent's child's parent is still itself
    // check 1)
    if(node->parent != nullptr) {
        double parent_cost = node->parent->cost_so_far;
        if (!abs(parent_cost + node->cost_to_parent - node->cost_so_far) < epsilon){
            std::cout <<  "ERROR:cost is wrong"<< std::endl;
            return false;
        }
    }
    else {
        std::cout <<  "ERROR:node parent does not exit"<< std::endl;
        return false;
    }
    // check 2) &
    // check for children
    RRT_Node *child_node = node->firstChild;
    while(child_node != nullptr){
        if (child_node->parent != node){
            std::cout <<  "ERROR:node's child node parent does not equal itself"<< std::endl;
            return false;
        }
        if(!check_node_correctness(child_node)){
            return false;
        }
        child_node = child_node->nextSibling;
    }
    return true;
}



int main(int argc, char *argv[]) {
    int thread_num = 1;
    int sample_num = 10000;
    if(argc > 1) {
        thread_num = std::atoi(argv[1]);
        if(argc > 2) {
            sample_num = std::atoi(argv[2]);
        }
    }
    std::cout << "Running with " << thread_num << " threads\n";
    srand(0);
    NaiveEnv env;
    RRT rrt(&env);
    rrt.options->gamma = 0.2;
    rrt.extend_radius = 0.01;
    Vector start = {0, 0}, goal = {1, 1};
    rrt.set_start_goal(start, goal);
    auto tnow = std::chrono::high_resolution_clock::now();
    for(int i = 0; i < 1; i++) {
        std::cout << "i = " << i << std::endl;
        if(rrt.plan_more(sample_num / thread_num, thread_num))
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
    return 0;
}
