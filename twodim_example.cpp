#include <iostream>
#include <fstream>
#include "prrtstar.h"

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

    bool is_clear(const double *v) {
        return true;
    }

    bool is_ingoal(const double *v) {
        return false;
    }
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
    printf("Debugging flag 0\n");
    
    std::vector<Env*> envs;
    for(int i = 0; i < thread_num; i++)
        envs.push_back(new NaiveEnv);
    RRT rrt;
    rrt.set_envs(&envs);
    rrt.options->gamma = 0.3;
    rrt.set_envs(&envs);
    NaiveEnv env;
    rrt.set_start_goal({0, 0}, {1, 1});

    auto tnow = std::chrono::high_resolution_clock::now();
    rrt.plan_more(sample_num / thread_num, thread_num);  // TODO: support incremental planning, cache memory
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