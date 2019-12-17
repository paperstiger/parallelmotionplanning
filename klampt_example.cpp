/*
implement the klampt example
*/
#include <iostream>
#include <fstream>
#include "prrtstar.h"

//Klampt stuff
#include <Klampt/Modeling/World.h>
#include <Klampt/Modeling/Robot.h>
#include <Klampt/Modeling/Terrain.h>
#include <Klampt/Planning/PlannerSettings.h>
#include <Klampt/Planning/RobotCSpace.h>

#define REGION_SPLIT_AXIS 0
#define DISCRETIZATION 0.01
class KlamptEnv : public Env{
public:
    std::shared_ptr<Robot> robot;
    RobotWorld world;
    gVector start;
    gVector goal = {0.0,-0.9228,0.4403,0.0,0.0,0.0};
    WorldPlannerSettings planner_settings;
    //SingleRobotCSpace* robot_CSpace;
    int index = 0;
    KlamptEnv() : Env(6){
        if(!world.LoadXML("klampt_data/tx90obstacles.xml")){
            printf("Error loading robot world");
        }
        robot = world.robots[0];
        auto tmp = robot->q;
        auto tmp_min = robot->qMin;
        auto tmp_max = robot->qMax;
        for (int i = 1;i<7;i++){
            start.push_back(tmp[i]);
            max.push_back(tmp_max[i]);
            min.push_back(tmp_min[i]); 
        }
        planner_settings.InitializeDefault(world);
    }

    bool is_clear(const double *v) {
        Config shared;
        shared.resize(7);
        shared[0] = 0;
        for(int i = 1;i<7;i++){
            shared[i] = v[i-1];}
        //copied from RobotCSpace.cpp
        robot->UpdateConfig(shared);
        robot->UpdateGeometry();
        int id = world.RobotID(index);
        vector<int> idrobot(1,id);
        vector<int> idothers;
        for(size_t i=0;i<world.terrains.size();i++)
            idothers.push_back(world.TerrainID(i));
        for(size_t i=0;i<world.rigidObjects.size();i++)
            idothers.push_back(world.RigidObjectID(i));
        for(size_t i=0;i<world.robots.size();i++) {
            if((int)i != index)
              idothers.push_back(world.RobotID(i));
          }
        //environment collision check
          pair<int,int> res = planner_settings.CheckCollision(world,idrobot,idothers);
          if(res.first >= 0) {
            //printf("Collision found: %s (%d) - %s (%d)\n",world.GetName(res.first).c_str(),res.first,world.GetName(res.second).c_str(),res.second);
            return false;
          }
          //self collision check
          res = planner_settings.CheckCollision(world,idrobot);
          if(res.first >= 0) {
            //printf("Self-collision found: %s %s\n",world.GetName(res.first).c_str(),world.GetName(res.second).c_str());
            return false;
          }
          return true;
    }

    bool is_free(const double *v1, const double *v2) {
        double d = _MyDistFun(v1,v2);
        double m[dim];
        int i;

        if (d < DISCRETIZATION) {
            return true;
        }

        for (i=0 ; i<dim ; ++i) {
                m[i] = (v1[i] + v2[i]) / 2.0;
                //printf("%f",m[i]);
        }

        return is_clear(m)
                && is_free(v1, m)
                && is_free(m, v2);

    }
    void sample(double *x, int id, int num) {
        if(num <= 1) {  // num if not greater than 1
            double *p = x;
            for(int i = 0; i < dim; i++) {
                p[i] = ((double)rand() / RAND_MAX * (max[i] - min[i]) + min[i]);
                //printf("%f",p[i]);
            }
        }
        else{
            for(int i=0;i<dim;i++){
                if(i==REGION_SPLIT_AXIS){
                    x[i] = (id + (double)rand() / RAND_MAX) / num * (max[i] - min[i]) + min[i];
                }
                else{
                    x[i] = ((double)rand() / RAND_MAX * (max[i] - min[i]) + min[i]);
                }
            }
        }
        //printf("\n");
    }

    double _MyDistFun(const double *a, const double *b)
    {
        double dist = 0;
        for(int i = 0; i < dim; i++)
            dist += pow(a[i] - b[i], 2);
        return sqrt(dist);
    }

    bool is_ingoal(const double *v) {
        return false;
    }

    ~KlamptEnv() {}
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
        envs.push_back(new KlamptEnv);
    RRT rrt;
    rrt.set_envs(&envs);
    rrt.options->gamma = 4.0;
    rrt.set_envs(&envs);
    KlamptEnv env;
    rrt.set_start_goal(env.start, env.goal);

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
    for(auto _env : envs)
       delete _env;
    return 0;
}