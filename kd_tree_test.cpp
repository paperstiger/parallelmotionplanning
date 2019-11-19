/*
Write a test function for KD tree.
I will randomly generate some points and append to the tree
*/
#include "KDTree.h"
#include "stdlib.h"
#include "time.h"
#include <iostream>

Vector random_gen_one(int dim) {
    Vector point(dim);
    for(int i = 0; i < dim; i++)
        point[i] = (double)rand() / RAND_MAX;
    return point;
}

int main() {
    srand(time(0));
    int dim = 5, num = 1000, init_num = 100, maxleafpoints = 30;  // add num points of dimension dim
    // first generate 100 points
    std::vector<Vector> points;
    for(int i = 0; i < init_num; i++)
        points.push_back(random_gen_one(dim));
    // construct the tree
    KDTree *tree = KDTree::Create(points, dim, 20);
    for(int i = 0; i < num; i++) {
        Vector tmp = random_gen_one(dim);
        tree->Insert(tmp, init_num + i, maxleafpoints);
    }
    // get some queries
    for(int i = 0; i < 10; i++) {
        std::cout << "Test " << i << std::endl;
        Vector tmp = random_gen_one(dim);
        std::vector<double> distances;
        std::vector<int> ids;
        tree->ClosePoints(tmp, 0.3, distances, ids);
        std::cout << distances.size() << " neighbors within 0.1\n";
        for(int j = 0; j < distances.size(); j++)
            std::cout << j << " " << ids[j] << " " << distances[j] << std::endl;
    }
    delete tree;
    return 0;
}