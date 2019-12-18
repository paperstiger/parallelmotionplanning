/*
implement of the nao example
*/
#include <iostream>
#include <fstream>
#include "prrtstar.h"

#include "collide.h"
#include "alloc.h"
#include "prrts.h"
#include "hrtimer.h"
#include "naocup.h"


#define REGION_SPLIT_AXIS 0
class NaocupEnv : public Env{
public:
    prrts_system_t *system;
    void *system_data;
    gVector start,goal;
    NaocupEnv() : Env(10){
        system = naocup_create_system();
        for(int i=0;i<dim;i++){
            min.push_back(system->min[i]);
            max.push_back(system->max[i]);
            start.push_back(system->init[i]);
            goal.push_back(system->target[i]);
        }
        system_data = (system->system_data_alloc_func)(0,system->min,system->max);
    }

    bool is_free(const double *v1, const double *v2) {
        return system->link_func(system_data,v1,v2);
    }

    bool is_clear(const double *v) {
        return system->clear_func(system_data, v);
    }

    bool is_ingoal(const double *v) {
        return system->in_goal_func(system_data, v);
    }

    void sample(double *x, int id, int num) {
        if(num <= 1) {  // num if not greater than 1
            double *p = x;
            for(int i = 0; i < dim; i++) {
                p[i] = (double)rand() / RAND_MAX * (max[i] - min[i]) + min[i];
            }
        }
        else{
            for(int i=0;i<dim;i++){
                if(i==REGION_SPLIT_AXIS){
                    x[i] = (id + (double)rand() / RAND_MAX) / num * (max[i] - min[i]) + min[i];
                }
                else{
                    x[i] = (double)rand() / RAND_MAX * (max[i] - min[i]) + min[i];
                }
            }
        }
    }
    ~NaocupEnv() {}
};


int main(int argc, char *argv[]) {
    int thread_num = 1;
    int sample_num = 100;
    if(argc > 1) {
        thread_num = std::atoi(argv[1]);
        if(argc > 2) {
            sample_num = std::atoi(argv[2]);
        }
    }
    std::cout << "Running with " << thread_num << " threads\n";
    srand(time(0));  // Yifan: uncomment this to have deterministic results
    
    std::vector<Env*> envs;
    for(int i = 0; i < thread_num; i++)
        envs.push_back(new NaocupEnv);
    RRT rrt;
    rrt.set_envs(&envs);
    rrt.options->gamma = 5.0;
    rrt.set_envs(&envs);
    NaocupEnv env;
    rrt.set_start_goal(env.start, env.goal);

    auto tnow = std::chrono::high_resolution_clock::now();
    // perform the planning
    rrt.plan_more(sample_num / thread_num, thread_num);

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
    for(auto _env : envs)
       delete _env;
    return 0;
}