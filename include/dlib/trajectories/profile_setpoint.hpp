#pragma once
#include "au/au.hpp"

namespace dlib {

template<
    typename Units
> requires 
    au::HasSameDimension<Units, au::Meters>::value || 
    au::HasSameDimension<Units, au::Radians>::value
struct ProfileSetpoint {
    au::Quantity<Units, double> position;
    au::Quantity<au::TimeDerivative<Units>, double> velocity;
    au::Quantity<au::Time2ndDerivative<Units>, double> acceleration;

    ProfileSetpoint(
        au::Quantity<Units, double> position, 
        au::Quantity<au::TimeDerivative<Units>, double> velocity,
        au::Quantity<au::Time2ndDerivative<Units>, double> acceleration
    ) : 
        position(position), 
        velocity(velocity),
        acceleration(acceleration) {

    };

    ProfileSetpoint direct(bool invert) const {
        if (invert) {
            return ProfileSetpoint(-position, -velocity, -acceleration);
        } else {
            return ProfileSetpoint(position, velocity, acceleration);
        }
    }
};


}