/*
Write serial version of RRT, just make everything work.
Besides, I switch to kdtree.h version since it may be useful
*/
#include <vector>
#include <list>
#include <iostream>
#include "stdlib.h"
#include "kdtree.h"
#include "arrayutils.h"

typedef std::vector<double> Vector;

// a base class for collision checker, it should be implemented by an environment
class CollisionChecker{
public:
    virtual bool is_free(const Vector &v1, const Vector &v2) = 0;
};

// a base class for an environment, it has to have to implement some functions
class Env : public CollisionChecker{
public:
    virtual Vector sample() = 0;

    Env(size_t dim_) : dim(dim_) {} 

    size_t dim;
    Vector min, max;
};

extern "C" double MyDistFun(const double *a, const double *b)
{
    double dist = 0;
    for(int i = 0; i < 2; i++)
        dist += pow(a[i] - b[i], 2);
    return sqrt(dist);
}

// this class maintains some cool stuff
template<class T>
class TreeNode{
public:
    T value;
    TreeNode *firstChild, *lastChild;
    TreeNode *nextSibling;
    TreeNode *parent;
    TreeNode(const T &val) : value(val),
                             firstChild(nullptr),
                             lastChild(nullptr),
                             nextSibling(nullptr),
                             parent(nullptr)
                             {}
    void add_child(TreeNode<T> *node) {
        if(!firstChild){
            firstChild = node;
        }
        else{
            lastChild->nextSibling = node;
        }
        lastChild = node;
        node->parent = this;
    }
};

class RRT{
public:
    double extend_radius;
    Vector start, goal;
    kd_tree_t *tree;
    Env *env;
    typedef TreeNode<Vector> RRT_Node;
    std::list<RRT_Node> all_nodes;

    void set_start_goal(const Vector &start_, const Vector &goal_) {
        start = start_;
        goal = goal_;
    }

    RRT(Env *env_) : tree(nullptr), env(env_), _goal_node(nullptr) {
        extend_radius = 0.1;
    }

    // following Kris convention, this function samples more
    bool plan_more(int num) {
        for(int i = 0; i < num; i++){
            if(!tree){
                std::vector<Vector> allp = {start};
                all_nodes.push_back(TreeNode<Vector>(start));
                tree = kd_create_tree(env->dim, env->min.data(), env->max.data(),
                          MyDistFun,
                          start.data(), &(all_nodes.back()));
            }
            // sample
            Vector x = env->sample();
            // find closest point
            double dist = 0;
            RRT_Node *close_node = (RRT_Node*)kd_nearest(tree, x.data(), &dist);
            // std::cout << "knn dist = " << dist << std::endl;
            if(dist > extend_radius)
                _extend_within(x, close_node->value, dist);
            // check if collision free
            if(env->is_free(x, close_node->value)){
                auto prev_node = close_node;
                all_nodes.push_back(TreeNode<Vector>(x));
                auto *new_node = &(all_nodes.back());
                prev_node->add_child(new_node);
                kd_insert(tree, all_nodes.back().value.data(), &(all_nodes.back()));
                // check if this node reaches goal
                if((ArrayUtils::Distance_L2(new_node->value, goal) < extend_radius) && (env->is_free(new_node->value, goal))){
                    // eventually we can add goal node
                    all_nodes.push_back(RRT_Node(goal));
                    _goal_node = &(all_nodes.back());
                    new_node->add_child(_goal_node);
                    return true;
                }
            }
        }
        return false;
    }

    // return the path
    std::vector<Vector> get_path() {
        std::vector<Vector> path;
        if(!_goal_node)
            return path;
        std::list<Vector> list_q;
        auto node = _goal_node;
        while(node) {
            list_q.push_front(node->value);
            node = node->parent;
        }
        for(auto iter=list_q.begin(); iter != list_q.end(); iter++)
            path.push_back(*iter);
        return path;
    }

    ~RRT() {
        delete tree;
    }
private:
    RRT_Node *_goal_node;

    void _extend_within(Vector &x, const Vector &base, double dist) const {
        double factor = extend_radius / dist;
        const Vector &pnt = base;
        for(size_t i = 0; i < x.size(); i++) {
            x[i] = pnt[i] + factor * (x[i] - pnt[i]);
        }
    }
};

// a naive environment simply from (0, 0) to (1, 1), no obstacle
class NaiveEnv : public Env {
public:
    NaiveEnv() : Env(2) {
        min = {0, 0};
        max = {1, 1};
    }

    bool is_free(const Vector &v1, const Vector &v2) {
        double x1 = v1[0], x2 = v2[0], y1 = v1[1], y2 = v2[1];
        if((x1 - 0.5) * (x2 - 0.5) >= 0)
            return true;
        else{
            double factor = (0.5 - x2) / (x1 - x2);
            double interpy = y2 + factor * (y1 - y2);
            return (interpy > 0.8) | (interpy < 0.2);
        }
    }

    Vector sample() {
        Vector p(dim);
        for(int i = 0; i < dim; i++) {
            p[i] = ((double)rand() / RAND_MAX * (max[i] - min[i]) + min[i]);
        }
        return p;
    }
};

int main() {
    srand(time(0));
    NaiveEnv env;
    RRT rrt(&env);
    Vector start = {0, 0}, goal = {1, 1};
    rrt.set_start_goal(start, goal);
    for(int i = 0; i < 10; i++) {
        std::cout << "i = " << i << std::endl;
        if(rrt.plan_more(100))
            break;
    }
    auto path = rrt.get_path();
    std::cout << "path length " << path.size() << std::endl;
    int stone = 0;
    for(auto &p : path) {
        std::cout << stone << " ";
        for(auto &n : p)
            std::cout << n << " ";
        std::cout << "\n";
        stone++;
    }
    return 0;
}
