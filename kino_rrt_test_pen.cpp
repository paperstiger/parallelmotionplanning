/*
Test the algorithm on pendulum problem
*/
#include "kino_tree_helper.h"

class PenGoalState : public StateSpace {
public:
    PenGoalState() : StateSpace(2) {
        upper << M_PI + 0.2, 0.2;
        lower << M_PI - 0.2, -0.2;
    }
};
// define state space for pendulum
class PenState : public StateSpace {
public:
    PenState() : StateSpace(2) {
        upper << 2 * M_PI, 8;
        lower << 0, -8;
    }

    bool valid(RefcVd state) {
        return true;
    }
};

// define control space for pendulum
class PenCtrl : public ControlSpace {
public:
    PenCtrl() : ControlSpace(1) {
        bangbang = false;
        upper << 2;
        lower << -2;
    }
};

class PenSys : public DynSystem {
private:
    PenGoalState pen_goal_state;
    PenState pen_state;
    PenCtrl pen_ctrl;
    double g, m, l;
public:
    PenSys() {
        pen_state.has_goal = true;
        pen_state.goal_space = &pen_goal_state;
        DynSystem::state = &pen_state;
        DynSystem::ctrl = &pen_ctrl;
        g = 9.8; m = 1; l = 1;
    }

    ~PenSys() {
    }

    Vd derivative(RefcVd state, RefcVd ctrl) {
        double th = state(0), thdot = state(1);
        double tau = ctrl(0);
        double thddot = (-3*g/(2*l) * sin(th + M_PI) + 3./(m*l*l)*tau);
        Vd deriv(2);
        deriv(0) = thdot;
        deriv(1) = thddot;
        return deriv;
    }

    double cost_derivative(RefcVd x, RefcVd u) {
        double angle = wrap_0_2pi(x(0)) - M_PI;
        return (angle * angle + 0.1 * x(1) * x(1) + 0.001 * u(0) * u(0));
        return 1;
    }

    double get_cost(RefcVd x0, RefcVd xf, RefcVd ctrl, double dt){
        double reward = 0;
        double dist_theta = fmod(xf(0), (2 * M_PI)) - M_PI;
        double dist_omega = fabs(xf(1));
        return -dt;
    }
};

int main_pendulum(int argc, char* argv[]) {
    // first step, define state
    Vd start(2);
    start << 0, 0;
    RRT rrt;
    rrt.sys = new PenSys();
    rrt.inte_max_dt = 0.5;
    rrt.set_start(start);
    L1Heuristic heu;
    rrt.heuristic = &heu;
    rrt.ctrl_sample_num = 8;
    bool plan_succeed = false;
    for(int i = 0; i < 10; i++) {
        std::cout << "Iter " << i << std::endl;
        plan_succeed = rrt.plan_more(20000);
        if(plan_succeed)
            break;
    }
    if(plan_succeed){
        std::cout << "Plan is successful\n";
        double *state_pnter = rrt.goal_node->state;
        for(int i = 0; i < rrt.sys->dimx(); i++)
            std::cout << state_pnter[i] << " ";
        std::cout << std::endl;
    }
    // memory
    delete rrt.sys;
    return 0;
}

int main_ao_pendulum(int argc, char* argv[]) {
    // first step, define state
    Vd start(2);
    start << 0, 0;
    RRT rrt;
    PenSys sys;
    rrt.sys = &sys;
    rrt.inte_max_dt = 0.5;
    rrt.set_start(start);
    L1Heuristic heu;
    rrt.heuristic = &heu;
    rrt.ctrl_sample_num = 3;
    CostSpaceRRT crrt(&rrt);
    bool plan_succeed = false;
    for(int i = 0; i < 10; i++) {
        std::cout << "Iter " << i << std::endl;
        plan_succeed = crrt.plan_more(20000);
    }
    if(plan_succeed)
        std::cout << "Plan is successful\n";
    if(rrt.goal_node){
        double *state_pnter = rrt.goal_node->state;
        for(int i = 0; i < rrt.sys->dimx(); i++)
            std::cout << state_pnter[i] << " ";
        std::cout << std::endl;
    }
    // memory
    return 0;
}