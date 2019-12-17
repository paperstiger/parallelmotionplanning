#ifndef PLANNER_COMMON_H
#define PLANNER_COMMON_H
#include <vector>

typedef std::vector<double> gVector;
// a base class for collision checker, it should be implemented by an environment
class CollisionChecker{
public:
    virtual bool is_free(const double *v1, const double *v2) = 0;
    virtual bool is_clear(const double *v1) = 0;
};

// a base class for an environment, it has to have to implement some functions
class Env{
public:
    virtual void sample(double *x, int id=-1, int num=-1) = 0;  // sample a configuration
    virtual bool is_free(const double *v1, const double *v2) = 0;  // check if the path from v1 to v2 is free
    virtual bool is_clear(const double *v1) = 0;  // check if a configuration is free
    virtual bool is_ingoal(const double *v1) = 0;  // check if a configuration is in_goal

    Env(size_t dim_) : dim(dim_) {
    } 

    virtual ~Env() {}

    size_t dim;
    gVector min, max;
};

struct RRTOption {
    double gamma = 1;
};
#endif