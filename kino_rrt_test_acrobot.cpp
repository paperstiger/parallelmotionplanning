/*
Test the algorithm on the acrobot problem.
This problem is not easy as the Python script suggests.
But C++ is really a boost.
*/
#include "kino_tree_helper.h"

class AcrobotGoalState : public StateSpace {
public:
    AcrobotGoalState() : StateSpace(4) {
        double angle_tol = 0.5, omega_tol = 0.5;
        upper << M_PI + angle_tol, angle_tol, omega_tol, omega_tol;
        lower << M_PI - angle_tol, -angle_tol, -omega_tol, -omega_tol;
    }
};

// define state space for pendulum
class AcrobotState : public StateSpace {
public:
    AcrobotState() : StateSpace(4) {
        upper << 2 * M_PI, M_PI, 10, 10;
        lower << 0, -M_PI, -10, -10;
    }

    void round(RefVd state) {
        state(0) = wrap_0_2pi(state(0));
        state(1) = wrap_neg_pos_pi(state(1));
    }

    bool valid(RefcVd state) {
        return true;
    }
};

// define control space for pendulum
class AcrobotCtrl : public ControlSpace {
public:
    AcrobotCtrl() : ControlSpace(1) {
        bangbang = true;
        upper << 1;
        lower << -1;
    }
};

class WrappedHeuristic : public Heuristic {
public:
    double so2_dist(double angle1, double angle2) {
        double angle = wrap_0_2pi(angle1 - angle2);
        if(angle > M_PI)
            return 2 * M_PI - angle;
        return angle;
    }
    double operator()(RefcVd x0, RefcVd u, RefcVd xf, RefcVd goal) {
        double dist1 = so2_dist(xf(0), goal(0));
        double dist2 = so2_dist(xf(1), goal(1));
        // return -(dist1 + dist2 + fabs(xf(2) - goal(2)) + fabs(xf(3) - goal(3)));
        return -sqrt(dist1 * dist1 + dist2 * dist2 + (xf(2) - goal(2)) * (xf(2) - goal(2)) + (xf(3) - goal(3)) * (xf(3) - goal(3)));
    }
};

class AcrobotSys : public DynSystem {
private:
    AcrobotGoalState pen_goal_state;
    AcrobotState pen_state;
    AcrobotCtrl pen_ctrl;
    double LINK_LENGTH_1 = 1., LINK_LENGTH_2 = 1., LINK_MASS_1 = 1., LINK_MASS_2 = 1.,
    LINK_COM_POS_1 = 0.5,
    LINK_COM_POS_2 = 0.5,
    LINK_MOI = 1.;
public:
    AcrobotSys() {
        pen_state.has_goal = true;
        pen_state.goal_space = &pen_goal_state;
        DynSystem::state = &pen_state;
        DynSystem::ctrl = &pen_ctrl;
    }

    ~AcrobotSys() {
    }

    Vd derivative(RefcVd state, RefcVd ctrl) {
        double m1 = LINK_MASS_1;
        double m2 = LINK_MASS_2;
        double l1 = LINK_LENGTH_1;
        double lc1 = LINK_COM_POS_1;
        double lc2 = LINK_COM_POS_2;
        double I1 = LINK_MOI;
        double I2 = LINK_MOI;
        double g = 9.8;
        double a = ctrl(0);
        double theta1 = state(0);
        double theta2 = state(1);
        double dtheta1 = state(2);
        double dtheta2 = state(3);
        double pi = M_PI;
        double d1 = m1 * lc1 * lc1 + m2 * 
            (l1 * l1 + lc2 * lc2 + 2 * l1 * lc2 * cos(theta2)) + I1 + I2;
        double d2 = m2 * (lc2 * lc2 + l1 * lc2 * cos(theta2)) + I2;
        double phi2 = m2 * lc2 * g * cos(theta1 + theta2 - pi / 2.);
        double phi1 = - m2 * l1 * lc2 * dtheta2 * dtheta2 * sin(theta2) 
               - 2 * m2 * l1 * lc2 * dtheta2 * dtheta1 * sin(theta2) 
            + (m1 * lc1 + m2 * l1) * g * cos(theta1 - pi / 2) + phi2;
        double    ddtheta2 = (a + d2 / d1 * phi1 - phi2) / 
                (m2 * lc2 * lc2 + I2 - d2 * d2 / d1);
        double ddtheta1 = -(d2 * ddtheta2 + phi1) / d1;
        Vd deriv(4);
        deriv << dtheta1, dtheta2, ddtheta1, ddtheta2;
        return deriv;
    }

    double cost_derivative(RefcVd state, RefcVd ctrl) {
        return 1;
    }
    double get_cost(RefcVd x0, RefcVd xf, RefcVd ctrl, double dt){
        double reward = 0;
        double dist_theta = fmod(xf(0), (2 * M_PI)) - M_PI;
        double dist_omega = fabs(xf(1));
        return -dt;
    }
};


int main_acrobot(int argc, char *argv[]) {
    // first step, define state
    std::cout << "Test acrobot\n";
    Vd start(4);
    start << 0, 0, 0, 0;
    RRT rrt;
    rrt.sys = new AcrobotSys();
    rrt.inte_max_dt = 0.5;
    rrt.set_start(start);
    // L1Heuristic heu;
    WrappedHeuristic heu;
    rrt.heuristic = &heu;
    rrt.ctrl_sample_num = 2;
    rrt.heuristic_use_goal = false;
    rrt.sample_rand_action_prob = 0.2;
    bool plan_succeed = false;
    for(int i = 0; i < 10; i++) {
        std::cout << "Iter " << i << std::endl;
        plan_succeed = rrt.plan_more(50000);
        if(plan_succeed)
            break;
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
    delete rrt.sys;
    return 0;
}

int main_ao_acrobot(int argc, char *argv[]) {
    // first step, define state
    std::cout << "Test acrobot\n";
    Vd start(4);
    start << 0, 0, 0, 0;
    RRT rrt;
    AcrobotSys acrobot;
    rrt.sys = &acrobot;
    rrt.inte_max_dt = 0.5;
    rrt.set_start(start);
    // L1Heuristic heu;
    WrappedHeuristic heu;
    rrt.heuristic = &heu;
    rrt.ctrl_sample_num = 2;
    rrt.heuristic_use_goal = false;
    rrt.sample_rand_action_prob = 0.2;
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
    return 0;
}