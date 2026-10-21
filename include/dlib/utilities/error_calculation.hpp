#pragma once
#include "au/au.hpp"

namespace dlib {

 /**
     * @brief Calculate linear error
     * 
     * @param target setpoint; where you want to end up
     * @param reading current displacement of your robot
     * 
     * @return error
     */
au::Quantity<au::Meters, double> linear_error(
    au::Quantity<au::Meters, double> target, 
    au::Quantity<au::Meters, double> reading
);

/**
     * @brief Make a clean angle that is more accurate
     * 
     * @param angle unsanitized angle
     * 
     * @return sanitized angle
     */
au::Quantity<au::Degrees, double> sanitize_angle(
    au::Quantity<au::Degrees, double> angle
);

/**
     * @brief Calculate angular error
     * 
     * @param target target heading
     * @param reading current heading of your robot
     * 
     * @return error
     */
au::Quantity<au::Degrees, double> angular_error(
    au::Quantity<au::Degrees, double> target, 
    au::Quantity<au::Degrees, double> reading
);

/**
     * @brief Set a relative target
     * 
     * @param start start displacement of your robot
     * @param target target displacement of your robot
     * 
     * @return error
     */
template<typename Units>
au::Quantity<Units, double> relative_target(
    au::Quantity<Units, double> start,
    au::Quantity<Units, double> target
) {
    return start + target;
}

}