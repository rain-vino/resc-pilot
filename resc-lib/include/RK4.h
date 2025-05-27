//
// Created by Zhaohong Liu on 24-9-3.
//

#ifndef MOGENLIB_RK4_H
#define MOGENLIB_RK4_H

#include <functional>
#include <vector>
#include <Eigen/Eigen>

using State = Eigen::Matrix<double, 12, 1>;

class RK4 {
public:
    template <typename EigenVecMat>
    static EigenVecMat rk4(
        const std::function<EigenVecMat(double, const EigenVecMat&)>& f,
        const EigenVecMat& y0, double t0, double tf, double h);

    static std::vector<double> rk4(
        const std::function<std::vector<double>(double, const std::vector<double>&)>& f,
        const std::vector<double>& y0, double t0, double tf, double h);

    static State rk4(
        const std::function<State(double, const State&)>& f,
        const State& y0, double t0, double tf, double h);
};

#endif //MOGENLIB_RK4_H
