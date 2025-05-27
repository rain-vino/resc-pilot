//
// Created by Zhaohong Liu on 24-9-10.
//

#include <iostream>

#include "RateControl.h"
#include "drone/DroneBase.h"
#include "drone/Iris.h"

int main() {
    std::shared_ptr<DroneBase> drone = std::make_shared<Iris>();
    RateControl controller(drone);

    Eigen::Vector3d current_body_rate(0.0, 0.0, 0.0);
    Eigen::Vector3d desired_body_rate(1.0, -1.0, 0.2);
    double collective_thrust = MogenParams::G * drone->mass_;

    auto torque =
            controller.update(current_body_rate, desired_body_rate, collective_thrust);

    std::cout << "torque: " << torque.transpose() << std::endl;

    controller.checkParams();
}
