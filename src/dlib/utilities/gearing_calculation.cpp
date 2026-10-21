#include "dlib/utilities/gearing_calculation.hpp"

namespace dlib {
    au::Quantity<au::Rpm, double> gearset_rpm(pros::MotorGearset gearset) {
        switch (gearset) {
            case pros::MotorGearset::red: return au::rpm(100); break;
            case pros::MotorGearset::green: return au::rpm(200); break;
            case pros::MotorGearset::blue: return au::rpm(600); break;
            case pros::MotorGearset::invalid: return au::rpm(0); break;
            default: return au::rpm(0); break;
        }
    }
}