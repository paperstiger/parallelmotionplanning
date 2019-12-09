#ifndef __KINO_TREE_NODE_H
#define __KINO_TREE_NODE_H
#include <limits>
#include <cstdlib>
#include <random>
#include <tuple>
#include "eigentypedef.h"
#include "kdtree.h"

// this section is devoted to random stuff
class ZeroOneRand {
public:
    ZeroOneRand(std::uint_fast32_t seed=std::numeric_limits<std::uint_fast32_t>::infinity()) {
        if(seed == std::numeric_limits<std::uint_fast32_t>::infinity()) {
            std::random_device r;
            seed = r();
        }
        e2 = new std::mt19937(seed);
        dist = new std::uniform_real_distribution<double>(0., 1.0);
    }

    double sample() {
        return dist->operator()(*e2);
    }

    ~ZeroOneRand() {
        delete e2;
        delete dist;
    }
private:
    std::mt19937 *e2;
    std::uniform_real_distribution<double> *dist;
};

class IntRangeRand {
public:
    IntRangeRand(int low, int upp, std::uint_fast32_t seed=std::numeric_limits<std::uint_fast32_t>::infinity()) {
        if(seed == std::numeric_limits<std::uint_fast32_t>::infinity()) {
            std::random_device r;
            seed = r();
        }
        e2 = new std::mt19937(seed);
        dist = new std::uniform_int_distribution<int>(low, upp);
    }

    double sample() {
        return dist->operator()(*e2);
    }

    ~IntRangeRand() {
        delete e2;
        delete dist;
    }
private:
    std::mt19937 *e2;
    std::uniform_int_distribution<int> *dist;
};

void set_random_seed(std::uint_fast32_t seed_);
double rand_zero_one();
Vd rand_zero_one(int n);
Vd rand_lb_ub(RefcVd lb, RefcVd ub);
int rand_binary();
int rand_triplet();

inline double wrap_0_2pi(double input) {
    return input - floor(input / (2 * M_PI)) * 2 * M_PI;
}

inline double wrap_neg_pos_pi(double input) {
    return wrap_0_2pi(input + M_PI) - M_PI;
}

// define a state space
// dimx: dimension of the space
// upper: array, the upper bounds for state
// lower: array, the lower bounds for state
// has_goal: bool, if this space has a goal
// goal_space: StateSpace *, if has_goal, the goal is defined so.
class StateSpace{
public:
    int dimx;
    Vd upper, lower;
    bool has_goal;
    StateSpace *goal_space;
    StateSpace(int dim) : dimx(dim), upper(dim), lower(dim), has_goal(false), goal_space(nullptr) {}

    virtual Vd sample();

    virtual bool valid(RefcVd state);

    virtual void round(RefVd state) {}

    virtual ~StateSpace() {}
};

// define a cost augmented state space
// it keeps a pointer to the space it is defined
// but itself is another StateSpace subclass
class CostStateSpace : public StateSpace{
private:
    StateSpace *_state;
    double cost_max;
    bool enable_cost_filter = false;
public:
    CostStateSpace(StateSpace *space) : StateSpace(space->dimx + 1), _state(space) {
        cost_max = std::numeric_limits<double>::infinity();
        has_goal = space->has_goal;
        upper.head(space->dimx) = space->upper;
        upper(space->dimx) = 10000;  // TODO: remove this magic number
        lower.head(space->dimx) = space->lower;
        lower(space->dimx) = 0;
    }

    void set_cost_max(double cost);

    double get_cost_max() const;

    Vd sample();

    bool valid(RefcVd state);

    void round(RefVd state);
};

class ControlSpace{
public:
    int dimu;
    bool bangbang;
    Vd upper, lower;
    ControlSpace(int dim) : dimu(dim), bangbang(false), upper(dim), lower(dim) {}
    virtual Vd sample();
    virtual ~ControlSpace() {}
};

// this class defines a dynamic system, some function has to be overloaded
class DynSystem{
public:
    DynSystem() : state(nullptr), ctrl(nullptr) {}

    DynSystem(StateSpace *state_, ControlSpace *ctrl_) : state(state_), ctrl(ctrl_) {}

    virtual Vd derivative(RefcVd state, RefcVd ctrl) = 0;  // return the derivative

    // virtual double get_cost(RefcVd x0, RefcVd xf, RefcVd ctrl, double dt);  // return cost function

    virtual double cost_derivative(RefcVd x0, RefcVd ctrl) = 0;

    virtual ~DynSystem() {};

    std::tuple<bool, Vd, double> integrate(RefcVd x0, RefcVd ctrl, double time, double dt=-1);  // integrate

    int dimx() {return state->dimx;}

    int dimu() {return ctrl->dimu;}

public:
    StateSpace *state;
    ControlSpace *ctrl;
};

// this class defines an extended dynamic system that includes the cost dimension
class CostDynSystem : public DynSystem {
private:
    DynSystem *_system;
    CostStateSpace wrapped_state_space;
    CostStateSpace wrapped_goal_space;
public:
    CostDynSystem(DynSystem *sys);

    Vd derivative(RefcVd state, RefcVd ctrl);  // return the derivative

    double cost_derivative(RefcVd x0, RefcVd ctrl);  // return cost function

    void update_cost_max(double);

    ~CostDynSystem();

};

// define a heuristic score for a triplet, x0, u, xf
class Heuristic{
public:
    virtual double operator()(RefcVd x0, RefcVd u, RefcVd xf, RefcVd goal) {
        return 0;
    };
};

// wrapper for Heuristic which neglects the last dimension
class CostHeuristic : public Heuristic{
private:
    Heuristic *_heuristic;
public:
    CostHeuristic(Heuristic *heu) : _heuristic(heu) {}
    double operator()(RefcVd x0, RefcVd u, RefcVd xf, RefcVd goal) {
        return _heuristic->operator()(x0.head(x0.size() - 1), u, xf.head(xf.size() - 1), goal.head(goal.size() - 1));
    }
};

class L1Heuristic : public Heuristic{
public:
    double operator()(RefcVd x0, RefcVd u, RefcVd xf, RefcVd goal) {
        double cost = 0;
        for(int i = 0; i < xf.size(); i++)
            cost -= fabs(xf(i) - goal(i));
        return cost;
    }
};

class L2Heuristic : public Heuristic{
public:
    double operator()(RefcVd x0, RefcVd u, RefcVd xf, RefcVd goal) {
        double cost = 0;
        for(int i = 0; i < xf.size(); i++)
            cost -= pow(xf(i) - goal(i), 2);
        return cost;
    }
};

struct Node {
    double *state = nullptr;  // the state at this node
    double *ctrl = nullptr;  // what control brings it here
    Node * volatile child = nullptr;
    Node * volatile sibling = nullptr;
    Node * volatile parent = nullptr;
    double reward_so_far = std::numeric_limits<double>::infinity();
    double dt = 0;

    void add_child(Node *node, double edgecost);
};

// define the memory manager
// with a given allocation size, it allocates enough memory to be later operated on
class memory_manager{
public:
    memory_manager(int basesize_, int dimx_, int dimu_) : size(basesize_), dimx(dimx_), dimu(dimu_), inner_count(0), prev(nullptr) {
        this->allocate_memory(size);
    }

    std::tuple<double*, double*, Node*, memory_manager*> get_data_node() {
        int cur_count = inner_count;
        if(cur_count == size) {
            memory_manager *next = new memory_manager(size, dimx, dimu);
            next->prev = this;
            return next->get_data_node();
        }
        else{
            inner_count += 1;
            return std::make_tuple(state_region + cur_count * dimx, ctrl_region + cur_count * dimu, (Node*)node_region + cur_count, this);
        }
    }

    void free_memory_manager() {
        memory_manager *curr = this;
        memory_manager *prev = nullptr;
        while(curr) {
            prev = curr->prev;
            free(curr->node_region);
            free((void*)curr->state_region);
            delete curr;
            curr = prev;
        }
    }

private:
    void allocate_memory(int size) {
        state_region = (double*)malloc(size * (dimx + dimu) * sizeof(double));
        ctrl_region = state_region + size * dimx;
        node_region = malloc(size * sizeof(Node));
    }

    int size, dimx, dimu;  // size of nodes and dimension of data
    memory_manager *prev;
    double *state_region;
    double *ctrl_region;
    void *node_region;
    int inner_count;
};

class RRT{
public:
    Vd start;
public:
    DynSystem *sys;
    Node *start_node, *goal_node;
    Heuristic *heuristic;
    int ctrl_sample_num = 3;  // this is used with heuristic
    bool heuristic_use_goal = false;
    bool last_dim_cost_hack = false;
    kd_tree_t *tree;
    memory_manager *manager;
    double sample_goal_prob = 0.1;
    double sample_rand_action_prob = 0.2;
    double inte_max_dt = 0.5;
    double inte_min_dt = 0.1;
    double max_reward;
    Node *max_reward_node;

    RRT() : sys(nullptr), start_node(nullptr), goal_node(nullptr), tree(nullptr),
     manager(nullptr), max_reward_node(nullptr), max_reward(-std::numeric_limits<double>::infinity()),
     heuristic(nullptr)
     {}

    void set_start(RefcVd start_) {
        start = start_;
    }

    bool heuristic_sample(RefcVd x0, RefVd ctrl, RefVd new_state, double &nodedt, double &edge_cost, RefcVd goal);

    int plan_more(int num);
    
    ~RRT() {
        kd_free(tree);
        manager->free_memory_manager();
        delete start_node;
    }
};

// define a new class that plans within cost space
class CostSpaceRRT {
private:
    RRT *_rrt;  // a pointer to existing RRT instance that's constructed as usual
    CostDynSystem *_cost_sys;
    CostHeuristic *_cost_heur;
public:
    CostSpaceRRT(RRT *rrt);

    // just plan for more iterations and see what happens
    bool plan_more(int num);

    ~CostSpaceRRT();
};

#endif