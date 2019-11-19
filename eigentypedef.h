/*
 * eigentypedef.h
 * Copyright (C) 2018 Gao Tang <gt70@duke.edu>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef EIGENTYPEDEF_H
#define EIGENTYPEDEF_H

/* I will define some useful typedef for eigen
 */
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include <iostream>

typedef Eigen::MatrixXd Md;
typedef Eigen::VectorXd Vd;
//typedef Eigen::VectorXi Vi;
typedef Eigen::VectorXi Vint;
//typedef Eigen::VectorXl Vi;
typedef Eigen::Matrix<long, -1, 1> Vl;
typedef Vl Vi;
typedef Eigen::Matrix<double, -1, -1, Eigen::RowMajor> RowMd;
typedef Eigen::Matrix<double, 1, -1, Eigen::RowMajor> RowVd;

template<typename T>
using EigenM = Eigen::Matrix<T, -1, -1>;
template<typename T>
using EigenRowM = Eigen::Matrix<T, -1, -1, Eigen::RowMajor>;
template<typename T>
using EigenV = Eigen::Matrix<T, -1, 1>;

typedef Eigen::Ref<Md> RefMd;
typedef Eigen::Ref<Vd> RefVd;
typedef Eigen::Ref<Vl> RefVi;
typedef Eigen::Ref<Vint> RefVint;
typedef Eigen::Ref<Vl> RefVl;
typedef Eigen::Ref<RowMd> RefRowMd;
typedef Eigen::Ref<RowVd> RefRowVd;

template<typename T>
using RefM = Eigen::Ref<Eigen::Matrix<T, -1, -1> >;
template<typename T>
using RefV = Eigen::Ref<Eigen::Matrix<T, -1, 1> >;

typedef Eigen::Ref<const Md> RefcMd;
typedef Eigen::Ref<const Vd> RefcVd;
typedef Eigen::Ref<const Vl> RefcVi;
typedef Eigen::Ref<const Vint> RefcVint;
typedef Eigen::Ref<const Vl> RefcVl;
typedef Eigen::Ref<const RowMd> RefcRowMd;

template<typename T>
using RefcM = Eigen::Ref<const Eigen::Matrix<T, -1, -1> >;
template<typename T>
using RefcV = Eigen::Ref<const Eigen::Matrix<T, -1, 1> >;

typedef Eigen::Map<Md> MapMd;
typedef Eigen::Map<Vd> MapVd;
typedef Eigen::Map<Vl> MapVi;
typedef Eigen::Map<Vint> MapVint;
typedef Eigen::Map<Vl> MapVl;
typedef Eigen::Map<RowMd> MapRowMd;
typedef Eigen::Map<const Vd> MapcVd;
typedef Eigen::Map<const Vint> MapcVi;

template<typename T>
using MapM = Eigen::Map<Eigen::Matrix<T, -1, -1> >;

typedef MapM<int> MapMi;

template<typename T>
using MapV = Eigen::Map<Eigen::Matrix<T, -1, 1> >;

template<typename T>
using SpM = Eigen::SparseMatrix<T>;

typedef SpM<double> SpMd;

typedef Eigen::Triplet<double> Tpt;
typedef Eigen::Triplet<int> Tpti;
typedef Eigen::Triplet<long> Tptl;

// this function prints the maximum row-col value of a sparse matrix
inline int getBandWidth(SpMd &mat){
    int sz = 0;
    for(int k = 0; k < mat.outerSize(); k++)
        for(SpMd::InnerIterator it(mat, k); it; ++it) {
            if(abs(it.row() - it.col()) > sz)
                sz = abs(it.row() - it.col());
        }
    return sz;
}

inline void printSpMdValue(SpMd &mat) {
    for(int k = 0; k < mat.outerSize(); k++){
        for(SpMd::InnerIterator it(mat, k); it; ++it) {
            std::cout << it.row() << " " << it.col() << " " << it.value() << ";";
        }
        std::cout << "\n";
    }
}
#endif /* !EIGENTYPEDEF_H */
