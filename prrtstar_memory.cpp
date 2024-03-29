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

#include "prrtstar.h"

//
#include <math.h>
//

// Ugly hack for dimension of the system
int __sys_dim__ = 0;
std::atomic<int> planned_num;
// std::condition_variable cv;
// int to_processed;
// std::atomic<int> finished;
const bool TEST_TREE_VALIDITY = false;

double MyDistFun(const double *a, const double *b)
{
    double dist = 0;
    for(int i = 0; i < __sys_dim__; i++)
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

// write subroutine for each thread
// problem is a pointer to the RRT instance
// found is a global indicator of exit, does not have to be atomic
// sample_num is per-thread sampling numbers
void thread_fun(RRT *problem, int sample_num, int);
bool check_node_correctness(RRT_Node *node);
bool check_tree_correctness(RRT_Node *start_node);


// following Kris convention, this function samples more
bool RRT::plan_more(int num, int thread_num) {
    if(!tree){
        assert(thread_num <= envs->size());
        start_node = new RRT_Node;
        start_node->value = start.data();
        start_node->cost_so_far = 0;  // I do not modify cost_to_parent since it has no parents
        __sys_dim__ = env->dim;
        tree = kd_create_tree(env->dim, env->min.data(), env->max.data(),
                    MyDistFun,
                    start.data(), start_node);
        goal_node = new RRT_Node;  // goal node is created but not in in kd_tree
        goal_node->value = goal.data();
        // kd_insert(tree, goal_node->value.data(), goal_node);  // RRT star has to start with two nodes
        tree_size.store(1, std::memory_order_relaxed);
    }
    std::vector<std::thread> threads;
    if(managers.size() == 0)  // haven't initialized yet
        managers.resize(thread_num);
    else
        assert(managers.size() == thread_num);
    //to_processed = thread_num;
    //finished.store(0);
    // this part controls how many we need total new nodes we need
    int total_plan_count = num * thread_num;
    planned_num.store(0);

    for(int i = 0; i < thread_num; i++) {
        threads.push_back(std::thread(thread_fun, this, total_plan_count, i));
    }
    for(auto &t : threads)
        t.join();
    if(TEST_TREE_VALIDITY)  // if anyone want to test tree validity
        this->test_tree_validity();
    return true;  // no exception raised means success
}

void RRT::update_goal_parent(RRT_Node *node, double radius, Env *_env) {
    double dist = MyDistFun(node->value, goal.data());
    if((dist > radius) || (!_env->is_free(node->value, goal.data()))) {
        node->cost_to_goal = std::numeric_limits<double>::infinity();
        return;
    }
    node->cost_to_goal = dist;  // cache this and collision check
    if(node->cost_so_far + dist < goal_node->cost_so_far) {
        //printf("up %f %f\n", node->cost_so_far + dist, goal_node->cost_so_far);
        goal_node->cost_so_far = node->cost_so_far + dist;
        goal_node->cost_to_parent = dist;
        goal_node->parent = node;
    }
}

// return the path
std::vector<gVector> RRT::get_path() {
    std::vector<gVector> path;
    if(goal_node->cost_so_far == std::numeric_limits<double>::infinity())
        return path;
    std::list<gVector> list_q;
    auto node = goal_node;
    while(node) {
        list_q.push_front(gVector(node->value, node->value + env->dim));
        node = node->parent;
    }
    for(auto iter=list_q.begin(); iter != list_q.end(); iter++)
        path.push_back(*iter);
    return path;
}

// do recursive way of visiting each stuff
// for each node, if it has children, we read root, read # nodes, and write nodes sequentially
void tree_helper(TreeNode *root, std::ofstream &off, int dim, double &edge_size) {
    if(root->firstChild) {
        double *start = root->value;
        off.write((char*)start, sizeof(double) * dim);
        TreeNode *node = root->firstChild;
        long child_size_buffer = off.tellp();
        double child_num = 0;
        off.write((char*)&child_num, sizeof(double));
        while(node) {
            off.write((char*)node->value, sizeof(double) * dim);
            edge_size += 1;
            node = node->nextSibling;
            child_num += 1;
        }
        long finish_child_buffer = off.tellp();
        off.seekp(child_size_buffer);
        off.write((char*)&child_num, sizeof(double));
        off.seekp(finish_child_buffer);
        // then recursive on each child...
        node = root->firstChild;
        while(node) {
            tree_helper(node, off, dim, edge_size);
            node = node->nextSibling;
        }
    }
}

// this function basically saves the tree into a file, wait, I have to somehow keep the memory alive...
// okay, do this now, and I can do plan_more whenever I want
void RRT::serialize(const std::string &fnm) {
    std::ofstream outfile(fnm, std::ofstream::binary);  // I will basically write everything down on this file, it may be huge though
    // first write size of dimension
    double dim = env->dim;
    double edge_size = 0;
    outfile.write((char*)&dim, sizeof(double));
    long edge_pos = outfile.tellp();
    outfile.write((char*)&edge_size, sizeof(double));
    tree_helper(start_node, outfile, dim, edge_size);
    long end_pos = outfile.tellp();
    outfile.seekp(edge_pos);
    outfile.write((char*)&edge_size, sizeof(double));
    outfile.seekp(end_pos);  // this kinda of finish serializing the tree
    // I still need to write the path, it should be quite straightforward
    auto path = get_path();
    // write size of the tree
    double path_size = path.size();
    outfile.write((char*)&path_size, sizeof(double));
    for(auto &node : path)
        outfile.write((char*)node.data(), sizeof(double) * dim);
    outfile.close();
}


void thread_fun(RRT *problem, int sample_num, int thread_id)
{
    int collision_check_counter = 0;
    Env *env = problem->envs->at(thread_id);
    memory_manager *mem_pnter = nullptr;
    int thread_num = problem->managers.size();
    int mean_expect_sample_num = sample_num / thread_num * 1.2;  // TODO: this 1.2 is somewhat a magic number
    if(problem->managers[thread_id] == nullptr) {
        mem_pnter = new memory_manager(mean_expect_sample_num, problem->env->dim);
        problem->managers[thread_id] = mem_pnter;
    }
    else
        mem_pnter = problem->managers[thread_id];
    auto &memory = *mem_pnter;
    list_pnter_dist nn_list;
    double goal_radius = 0.2;
    double *cmndata;
    RRT_Node *cmnnode;
    double *x = new double[problem->env->dim];
    int DIM = problem->env->dim;
    while(planned_num.load() < sample_num){
        env->sample(x, thread_id, thread_num);
        if(!env->is_clear(x)) {
            continue;
        }
        // find closest point
        double dist = 0;
        int step_no = problem->tree_size.load();
        double radius = problem->options->gamma *  // the radius being queried
                pow(log((double)step_no + 1.0) / (double)(step_no + 1.0),
                    1.0 / (double)problem->env->dim);
        goal_radius = radius;
        //std::cout << "radius = " << radius << "\n";
        nn_list.clear();
        int near_list_size = kd_near(problem->tree, x, radius, kd_near_call_back, &nn_list); // Update to keep path cost, not distance
        int _to_rewire_num = near_list_size;   // how many rewires do we need
        //printf("%d %f \n", step_no, x[0]);
        // in two conditions, we switch to nearest neighbor search
        // 1. no node found in the vinicity
        // 2. one node is found but is the goal node; the goal node shall not have any children
        bool new_node_created = false;
        int nn_index = 0;
        if((near_list_size == 0)) {
            double nearest_dist = 0;
            RRT_Node *near_node = (RRT_Node*)kd_nearest(problem->tree, x, &nearest_dist);
            // extend x so that its distance is exact radius
            problem->_extend_within(x, near_node->value, radius / nearest_dist);
            // check collision
            collision_check_counter++;
            if(!env->is_free(x, near_node->value)) {  // sample is infeasible from near_node
                continue;
            }
            // create node and return, no rewiring is needed
            planned_num.fetch_add(1, std::memory_order_relaxed);
            std::tie(cmndata, cmnnode) = memory.get_data_node();
            std::memcpy(cmndata, x, DIM * sizeof(double));
            cmnnode->init(cmndata);
            RRT_Node *new_node = cmnnode;
            near_node->add_child(new_node, radius);
            kd_insert(problem->tree, new_node->value, new_node);
            problem->tree_size.fetch_add(1, std::memory_order_relaxed);
            // check if goal can be routed here
            problem->update_goal_parent(new_node, goal_radius, env);
        }
        else{  // link new node to the nearest one and perform rewiring based on that
            // first find the closest node to connect into
            nn_list.sort([](const pvd &n1, const pvd &n2) {return ((RRT_Node*)(n1.first))->cost_so_far + n1.second < ((RRT_Node*)(n2.first))->cost_so_far + n2.second;});
            //std::cout << "rewire " << lst_pnt_dis.size() << " nodes\n";
            for(auto it = nn_list.begin(); it != nn_list.end(); it++) {
                _to_rewire_num--;  // this one should be skipped
                RRT_Node *cand = (RRT_Node*)(it->first);
                //printf("nn id %d q %f dist %f\n", nn_index, cand->value[0], it->second);
                nn_index++;
                //if(cand == problem->goal_node)  // this is the goal node, we rewire based on if its has parent
                //    continue;  // remove because goal is never in the tree
                collision_check_counter++;
                if(env->is_free(x, cand->value)) {  //  collision free, can add node
                    planned_num.fetch_add(1, std::memory_order_relaxed);
                    std::tie(cmndata, cmnnode) = memory.get_data_node();
                    new_node_created = true;
                    std::memcpy(cmndata, x, DIM * sizeof(double));
                    cmnnode->init(cmndata);
                    RRT_Node *new_node = cmnnode;
                    cand->add_child(new_node, it->second);
                    problem->update_goal_parent(new_node, goal_radius, env);
                    kd_insert(problem->tree, new_node->value, new_node);
                    problem->tree_size.fetch_add(1, std::memory_order_relaxed);
                    // try to rewire
                    if(_to_rewire_num > 0) {
                        for (auto rit = nn_list.rbegin(); rit != nn_list.rend(); rit++) {
                            if(_to_rewire_num == 0)
                                break;
                            //printf("rewire %d %d\n", i, _to_rewire_num);
                            _to_rewire_num--;
                            RRT_Node *cani = (RRT_Node*)(rit->first);
                            double cani_dist = rit->second;
                            // rewire only if gets a shorter path
                            if(cani_dist + new_node->cost_so_far >= cani->cost_so_far) {
                                continue;
                            }
                            // collision free has to be satisfied
                            collision_check_counter++;
                            if(!env->is_free(x, cani->value))
                                continue;
                            // rewiring is needed
                            // we need to delete this node, rewire to another node
                            // traversal needs to make sure prev->next is itself
                            // delete needs to make sure prev and next are the same
                            // first, search for cani in cani's parent's children
                            RRT_Node *parent = cani->parent;
                            bool rmflag = parent->remove_child(cani);
                            if(!rmflag){
                                continue;  // TODO: may remove cani from its new parents if possible
                            }
                            new_node->add_child(cani, cani_dist);
                            // hopefully we can play with cani now
                            double new_cost = cani->cost_so_far;
                            cani->update_cost(new_cost, problem->goal_node);
                        }
                    }
                    break;
                }
            }
        }
    }
    printf("number of collision checks in thread %d is %d\n", thread_id, collision_check_counter);
    delete[] x;
    // finished.fetch_add(1);
    // std::unique_lock<std::mutex> lk(problem->m);
    // cv.wait(lk, []{return to_processed == finished;});
    // lk.unlock();
    // cv.notify_one();
}

bool RRT::test_tree_validity() {
    if (check_tree_correctness(this->start_node)){
        std::cout << "Tree Valid " << std::endl;
    }
    else{
        std::cout << "Tree InValid " << std::endl;
    }
    int thread_id = 0;
    for(auto memptr : this->managers){
        int counter = 0;
        memory_manager &memory = *memptr;
        int mem_size = memory.get_size();
        memory.reset(); 
        double *tmpdata;
        RRT_Node *tmpnode;
        for (int i = 0; i < mem_size ; i++){    
            std::tie(tmpdata, tmpnode) = memory.get_data_node();
            if (tmpnode->parent == nullptr) {
                counter++;
            }
            else if (fabs(tmpnode->parent->cost_so_far + tmpnode->cost_to_parent - tmpnode->cost_so_far) > 1e-10)
            {
                counter++;
            }
        }
        std::cout << "thread:"<< " " <<thread_id++  << " " << "has " << counter << " " << "have parental issues" << std::endl;
    }
}

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