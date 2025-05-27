//
// Created by Zhaohong Liu on 24-9-3.
//

#include <iostream>
#include <vector>
#include <RK4.h>

std::vector<double> example_f([[maybe_unused]] double t, const std::vector<double>& y) {
    std::vector<double> dydt(y.size());
    // Example differential equation: dy/dt = -y (exponential decay)
    for (size_t i = 0; i < y.size(); ++i) {
        dydt[i] = -y[i];
    }
    return dydt;
}

int main() {
    std::vector<double> y0 = {1.0};  // Initial condition y(t0) = 1
    double t0 = 0.0;
    double tf = 2.0;
    double h = 0.1;

    std::vector<double> result = RK4::rk4(example_f, y0, t0, tf, h);

    std::cout << "Result: ";
    for (const double& value : result) {
        std::cout << value << " ";
    }
    std::cout << std::endl;

    return 0;
}