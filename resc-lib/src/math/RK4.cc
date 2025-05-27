//
// Created by Zhaohong Liu on 24-9-3.
//

#include "RK4.h"

template<typename EigenVecMat>
EigenVecMat
RK4::rk4(const std::function<EigenVecMat(double, const EigenVecMat &)> &f,
         const EigenVecMat &y0, const double t0, const double tf, double h) {
    double t = t0;
    EigenVecMat y = y0;
    EigenVecMat k1, k2, k3, k4, y_temp;

    while (t < tf) {
        k1 = f(t, y);

        y_temp = y + h / 2 * k1;
        k2 = f(t + h / 2, y_temp);

        y_temp = y + h / 2 * k2;
        k3 = f(t + h / 2, y_temp);

        y_temp = y + h * k3;
        k4 = f(t + h, y_temp);

        y += h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);

        t += h;
    }

    return y;
}

State RK4::rk4(const std::function<State(double, const State &)> &f,
               const State &y0, const double t0, const double tf, const double h) {
    double t = t0;
    State y = y0;

    while (t < tf) {
        State k1 = f(t, y);

        State y_temp = y + h / 2 * k1;
        State k2 = f(t + h / 2, y_temp);

        y_temp = y + h / 2 * k2;
        State k3 = f(t + h / 2, y_temp);

        y_temp = y + h * k3;
        State k4 = f(t + h, y_temp);

        y += h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);

        t += h;
    }

    return y;
}


std::vector<double>
RK4::rk4(const std::function<std::vector<double>(double, const std::vector<double> &)>& f,
         const std::vector<double> &y0, double t0, double tf, double h) {
    double t = t0;
    std::vector<double> y = y0;
    std::vector<double> k1, k2, k3, k4, y_temp;

    while (t < tf) {
        k1 = f(t, y);

        y_temp = y;
        for (size_t i = 0; i < y.size(); ++i) {
            y_temp[i] += h / 2 * k1[i];
        }
        k2 = f(t + h / 2, y_temp);

        y_temp = y;
        for (size_t i = 0; i < y.size(); ++i) {
            y_temp[i] += h / 2 * k2[i];
        }
        k3 = f(t + h / 2, y_temp);

        y_temp = y;
        for (size_t i = 0; i < y.size(); ++i) {
            y_temp[i] += h * k3[i];
        }
        k4 = f(t + h, y_temp);

        for (size_t i = 0; i < y.size(); ++i) {
            y[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        }

        t += h;
    }

    return y;
}
