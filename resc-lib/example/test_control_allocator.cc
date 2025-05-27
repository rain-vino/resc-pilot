//
// Created by Zhaohong Liu on 24-11-19.
//

#include "drone/DroneBase.h"
#include "Convertor.h"
#include "ControlAllocator.h"
#include "drone/Iris.h"

int main() {
    std::shared_ptr<DroneBase> drone = std::make_shared<Iris>();
    ControlAllocator control_allocator(drone);

    const auto& effectiveness = control_allocator.getEffectiveness();
    std::cout << "effectiveness: " << std::endl;
    Convertor::printMatrix(effectiveness);

    const auto& mix = control_allocator.getMix();
    std::cout << "mix: " << std::endl;
    Convertor::printMatrix(mix);

    // example of how to use control allocator
    auto torque_flu_sp = Eigen::Vector3d(0.2, 0.0, 0.0);
    double thrust_flu_sp = 0.5 * 6.925;

    control_allocator.setControlSetpoint(torque_flu_sp, thrust_flu_sp);
    control_allocator.pseudoInverseAllocate();
    auto motor_thrusts = control_allocator.getMotorThrusts();

    std::cout << "Motor thrusts: " << motor_thrusts.transpose() << std::endl;

    return 0;
}
