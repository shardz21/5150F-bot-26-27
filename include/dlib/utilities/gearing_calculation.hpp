#pragma once

#include "au/au.hpp"
#include "pros/abstract_motor.hpp"
namespace dlib {

/**
     * @brief Get the RPM of the robot
     * 
     * @param gearset the gearset of the robot
     * 
     * @return RPM
     */

au::Quantity<au::Rpm, double> gearset_rpm(pros::MotorGearset gearset);

}