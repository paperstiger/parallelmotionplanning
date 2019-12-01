#ifndef PLANNER_COMMON_H
#define PLANNER_COMMON_H
#include <vector>

typedef std::vector<double> Vector;
// a base class for collision checker, it should be implemented by an environment
class CollisionChecker{
public:
    virtual bool is_free(const double *v1, const double *v2) = 0;
};

// a base class for an environment, it has to have to implement some functions
class Env : public CollisionChecker{
public:
    virtual void sample(double *x) = 0;

    Env(size_t dim_) : dim(dim_) {
    } 

    size_t dim;
    Vector min, max;
};

struct RRTOption {
    double gamma = 1;
};
#endif