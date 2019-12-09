#include "kino_tree_helper.h"
#include <random>
#include <set>
#include <algorithm>
#include <cmath>

void Node::add_child(Node *node, double cost) {
    if(child == nullptr)
        child = node;
    else {
        node->sibling = child;
        child = node;
    }
    child->reward_so_far = reward_so_far + cost;
}

// this section is devoted to random section
std::uint_fast32_t __rand_seed__ = std::numeric_limits<std::uint_fast32_t>::infinity();

void set_random_seed(std::uint_fast32_t seed_) {
    __rand_seed__ = seed_;
}

double rand_zero_one() {
    static ZeroOneRand seeder(__rand_seed__);
    return seeder.sample();
}

Vd rand_zero_one(int n) {
    Vd rst(n);
    for(int i = 0; i < n; i++)
        rst(i) = rand_zero_one();
}

Vd rand_lb_ub(RefcVd lb, RefcVd ub) {
    assert(lb.size() == ub.size());
    Vd rst(lb.size());
    for(int i = 0; i < rst.size(); i++) {
        rst(i) = rand_zero_one() * (ub(i) - lb(i)) + lb(i);
    }
    return rst;
}

int rand_binary() {
    static IntRangeRand seeder(0, 1, __rand_seed__);
    return seeder.sample();
}

int rand_triplet() {
    static IntRangeRand seeder(0, 2, __rand_seed__);
    return seeder.sample();
}

// section for space

Vd StateSpace::sample() {
    return rand_lb_ub(lower, upper);
}

bool StateSpace::valid(RefcVd state) {
    for(int i = 0; i < dimx; i++) {
        if((state(i) > upper(i)) || (state(i) < lower(i)))
            return false;
    }
    return true;
}

void CostStateSpace::set_cost_max(double cost) {
    cost_max = cost;
    enable_cost_filter = true;
}

double CostStateSpace::get_cost_max() const {
    return cost_max;
}

Vd CostStateSpace::sample() {
    Vd sample(dimx);
    sample.head(dimx - 1) = _state->sample();
    if(cost_max == std::numeric_limits<double>::infinity())
        sample(dimx - 1) = 0;
    else
        sample(dimx - 1) = rand_zero_one() * cost_max;
    return sample;
}

bool CostStateSpace::valid(RefcVd state) {
    if(!enable_cost_filter)
        return _state->valid(state.head(dimx - 1));
    else
        return _state->valid(state.head(dimx - 1)) && (state(dimx - 1) < cost_max);
}

void CostStateSpace::round(RefVd state) {
    _state->round(state.head(dimx - 1));
}

Vd ControlSpace::sample() {
    if(!bangbang)
        return rand_lb_ub(lower, upper);
    else{
        Vd control(dimu);
        for(int i = 0; i < dimu; i++) {
            int sam = rand_triplet();
            if(sam == 0)
                control(i) = lower(i);
            else if(sam == 1)
                control(i) = 0;
            else
                control(i) = upper(i);
        }
        return control;
    }
}

// return a cost function
// double DynSystem::get_cost(RefcVd x0, RefcVd xf, RefcVd ctrl, double dt) {
//     return 0;
// }

std::tuple<bool, Vd, double> DynSystem::integrate(RefcVd x0, RefcVd ctrl, double time, double dt) {
    Vd xf = x0;
    double leftt = time;
    double edge_cost = 0;
    while(leftt > 0) {
        double intt = std::min(dt, leftt);
        Vd deriv = derivative(xf, ctrl);
        double cost_deriv = cost_derivative(xf, ctrl);
        xf += deriv * intt;
        edge_cost += cost_deriv * intt;
        if(!state->valid(xf))
            return std::make_tuple(false, xf, edge_cost);
        leftt -= dt;
    }
    return std::make_tuple(true, xf, edge_cost);
}

CostDynSystem::CostDynSystem(DynSystem *sys) : wrapped_state_space(sys->state), wrapped_goal_space(sys->state->goal_space) {
    _system = sys;
    state = &wrapped_state_space;
    ctrl = sys->ctrl;
    state->has_goal = true; //TODO: remove this part since it may not be true
    state->goal_space = &wrapped_goal_space;
}

void CostDynSystem::update_cost_max(double cost) {
    wrapped_goal_space.set_cost_max(cost);
}

CostDynSystem::~CostDynSystem() {
}

Vd CostDynSystem::derivative(RefcVd state, RefcVd ctrl) {
    int dimx = this->dimx();
    Vd deriv(dimx);
    deriv.head(dimx - 1) = _system->derivative(state.head(dimx - 1), ctrl);
    deriv(dimx - 1) = _system->cost_derivative(state.head(dimx - 1), ctrl);
    return deriv;
}  // return the derivative

double CostDynSystem::cost_derivative(RefcVd x0, RefcVd ctrl) {
    int dimx = this->dimx();
    return _system->cost_derivative(x0.head(dimx - 1), ctrl);
}

double __so2_dist__(double angle1, double angle2) {
    double angle = wrap_0_2pi(angle1 - angle2);
    if(angle > M_PI)
        return 2 * M_PI - angle;
    return angle;
}

int __state_dim__;
double MyDistFun(const double *a, const double *b)
{
    double dist = 0;
    for(int i = 0; i < __state_dim__; i++)
        dist += pow(a[i] - b[i], 2);
    // dist += pow(__so2_dist__(a[0], b[0]), 2.);
    // dist += pow(__so2_dist__(a[1], b[1]), 2);
    // dist += pow(a[2] - b[2], 2);
    // dist += pow(a[3] - b[3], 2); 
    return sqrt(dist);
}


int RRT::plan_more(int num) {
    assert(sys->state->has_goal);
    if(!tree) {
        start_node = new Node;
        start_node->state = start.data();
        start_node->reward_so_far = 0;
        __state_dim__ = sys->dimx(); //FIXME: this is ugly hack
        if(last_dim_cost_hack)
            __state_dim__ -= 1;
        tree = kd_create_tree(__state_dim__, sys->state->lower.data(), sys->state->upper.data(), MyDistFun, start.data(), start_node);
        manager = new memory_manager(num, sys->dimx(), sys->dimu());
    }
    double *cmnx, *cmnu;
    Node *cmnnode;
    int dimx = sys->dimx(), dimu = sys->dimu();
    Vd state(dimx), new_state(dimx), ctrl(dimu);
    for(int i = 0; i < num; i++) {
        // sample a state randomly
        if(rand_zero_one() < sample_goal_prob)
            state = sys->state->goal_space->sample();
        else
            state = sys->state->sample();
        // find closest neighbor
        double nearest_dist;
        Node *near_node = (Node*)kd_nearest(tree, state.data(), &nearest_dist);
        MapVd x0(near_node->state, dimx);
        // sample control, we may have metrics to choose action
        bool inte_flag = false;
        double node_dt, edge_cost;
        if(heuristic_use_goal)
            inte_flag = heuristic_sample(x0, ctrl, new_state, node_dt, edge_cost, sys->state->goal_space->sample());
        else
            inte_flag = heuristic_sample(x0, ctrl, new_state, node_dt, edge_cost, state);
        if(!inte_flag) {
            std::cout << "integration error occurs\n";
        }
        //std::cout << "ctrl " << ctrl(0) << ";x0=" << x0(0) << " " << x0(1) << ";xf=" << new_state(0) << " " << new_state(1) << "\n";
        if(!inte_flag)  // infeasible control command
            continue;
        std::tie(cmnx, cmnu, cmnnode, manager) = manager->get_data_node();
        std::memcpy(cmnx, new_state.data(), dimx * sizeof(double));
        std::memcpy(cmnu, ctrl.data(), dimu * sizeof(double));
        cmnnode->state = cmnx;
        cmnnode->dt = node_dt;
        near_node->add_child(cmnnode, edge_cost);
        // insert this state into the tree
        kd_insert(tree, cmnnode->state, cmnnode);
        // optionally update the best_node, it may not be a leaf TODO: add this feature
        // if(cmnnode->reward_so_far > max_reward) {
        //     max_reward = cmnnode->reward_so_far;
        //     max_reward_node = cmnnode;
        // }
        // return if we found the new child is in goal
        //std::cout << new_state(0) << " " << new_state(1) << " " << new_state(2) << "\n";
        if(sys->state->goal_space->valid(new_state)) {
            goal_node = cmnnode;
            return i;
        }
    }
    return -1;
}

bool RRT::heuristic_sample(RefcVd x0, RefVd ctrl, RefVd new_state, double &nodedt, double &edge_cost, RefcVd goal) {
    nodedt = inte_max_dt * pow(rand_zero_one(), 1.0 / sys->ctrl->dimu);
    edge_cost = 0;
    if((!heuristic) || (rand_zero_one() < sample_rand_action_prob)) {
        ctrl = sys->ctrl->sample();
        bool inte_flag = false;
        std::tie(inte_flag, new_state, edge_cost) = sys->integrate(x0, ctrl, nodedt, inte_min_dt);
        sys->state->round(new_state);
        return inte_flag;
    }
    assert(ctrl_sample_num > 0);
    //std::set<Vd> *sampled = nullptr;
    if(sys->ctrl->bangbang) {
    //    sampled = new std::set<Vd>;
    }
    bool inte_flag;
    Vd tmp_state(sys->dimx());
    //static double best_val = -std::numeric_limits<double>::infinity();
    double best_heuristic = -std::numeric_limits<double>::infinity();
    for(int i = 0; i < ctrl_sample_num; i++) {
        Vd sample_ctrl = sys->ctrl->sample();
        //if(sampled){
        //    if(sampled->count(sample_ctrl) > 0)
        //        continue;
        //    else
        //        sampled->insert(sample_ctrl);
        //}
        std::tie(inte_flag, tmp_state, edge_cost) = sys->integrate(x0, sample_ctrl, nodedt, inte_min_dt);
        if(!inte_flag)
            std::cout << "fail to integrate\n";
        if(inte_flag) {
            sys->state->round(tmp_state);
            double new_heur = heuristic->operator()(x0, sample_ctrl, tmp_state, goal);
            if(new_heur > best_heuristic){
                best_heuristic = new_heur;
                ctrl = sample_ctrl;
                new_state = tmp_state;
            }
        }
    }
    // if(best_val < best_heuristic) {
    //     best_val = best_heuristic;
    //     std::cout << "best heu so far " << best_val << std::endl;
    // }
    // delete the dictionary
    //if(sampled)
    //    delete sampled;
    return (best_heuristic > -std::numeric_limits<double>::infinity());
}


// for CostSpaceRRT
CostSpaceRRT::CostSpaceRRT(RRT *rrt) : _rrt(rrt) {
    _cost_sys = new CostDynSystem(rrt->sys);
    _cost_heur = new CostHeuristic(rrt->heuristic);
    // replace the stuff in rrt with new ones
    rrt->sys = _cost_sys;
    // replace the start node
    Vd new_start(rrt->start.size() + 1);
    new_start.head(rrt->start.size()) = rrt->start;
    new_start.tail(1)(0) = 0;
    rrt->start.resize(new_start.size());
    rrt->start = new_start;
    rrt->heuristic = _cost_heur; 
    rrt->last_dim_cost_hack = true;
}

// let the cost space rrt plan more
bool CostSpaceRRT::plan_more(int num) {
    bool succeed = false;
    for(int i = 0; i < num; i++){
        int planned = _rrt->plan_more(1);
        if(planned == 0) {  // means success
            succeed = true;
            double cur_cost = _rrt->goal_node->state[_cost_sys->dimx() - 1];
            std::cout << "cur cost is " << cur_cost << "\n";
            _cost_sys->update_cost_max(cur_cost);
        }
    }
    return succeed;
}

CostSpaceRRT::~CostSpaceRRT() {
    delete _cost_sys;
    delete _cost_heur;
}