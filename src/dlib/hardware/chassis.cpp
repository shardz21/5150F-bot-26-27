#include "dlib/hardware/chassis.hpp"
#include "au/au.hpp"
#include "dlib/utilities/gearing_calculation.hpp"
#include "motor_group.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"

namespace dlib {

ChassisConfig::ChassisConfig(
    MotorGroupConfig left_motors_config,
    MotorGroupConfig right_motors_config,
    pros::MotorGearset drive_gearset,
    double gear_ratio,
    au::Quantity<au::Meters, double> wheel_diameter
)  :
    left_motors(left_motors_config),
    right_motors(right_motors_config),
    drive_gearset(drive_gearset),
    gear_ratio(gear_ratio),
    wheel_diameter(wheel_diameter) {

};

ChassisConfig ChassisConfig::from_drive_rpm(
    MotorGroupConfig left_motors_config, 
    MotorGroupConfig right_motors_config, 
    pros::MotorGearset drive_gearset, 
    au::Quantity<au::Rpm, double> total_rpm, 
    au::Quantity<au::Meters, double> wheel_diameter
) {
    auto base_rpm = gearset_rpm(drive_gearset);
    auto gear_ratio = total_rpm / base_rpm;

    return ChassisConfig(left_motors_config, right_motors_config, drive_gearset, gear_ratio, wheel_diameter);
}

ChassisConfig ChassisConfig::from_gear_ratio(
    MotorGroupConfig left_motors_config, 
    MotorGroupConfig right_motors_config, 
    pros::MotorGearset drive_gearset, 
    double gear_ratio, 
    au::Quantity<au::Meters, double> wheel_diameter
) {
    return ChassisConfig(left_motors_config, right_motors_config, drive_gearset, gear_ratio, wheel_diameter);
}

Chassis::Chassis(
    ChassisConfig config
) : 
    left_motors(config.left_motors), 
    right_motors(config.right_motors), 
    drive_gearset(config.drive_gearset),
    gear_ratio(config.gear_ratio),
    wheel_diameter(config.wheel_diameter) {
    

}

void Chassis::initialize() {
    left_motors.raw.tare_position_all();
    right_motors.raw.tare_position_all();

    left_motors.raw.set_encoder_units_all(pros::MotorUnits::rotations);
    right_motors.raw.set_encoder_units_all(pros::MotorUnits::rotations);

    left_motors.raw.set_gearing_all(drive_gearset);
    right_motors.raw.set_gearing_all(drive_gearset);
}

void Chassis::move(int32_t power) {
    left_motors.move(power);
    right_motors.move(power);
}

void Chassis::move_voltage(au::Quantity<au::Volts, double> voltage) {
    left_motors.move_voltage(voltage);
    right_motors.move_voltage(voltage);
}

void Chassis::turn(int32_t power) {
    left_motors.move(-power);
    right_motors.move(power);
}

void Chassis::turn_voltage(au::Quantity<au::Volts, double> voltage) {
    left_motors.move_voltage(voltage);
    right_motors.move_voltage(-voltage);
}

void Chassis::arcade(int32_t power, int32_t turn) {
    left_motors.move(power - turn);
    right_motors.move(power + turn);
}

void Chassis::brake() {
    left_motors.raw.brake();
    right_motors.raw.brake();
}

au::Quantity<au::Meters, double> Chassis::revolutions_to_displacement(au::Quantity<au::Revolutions, double> revolutions) const {
    // TODO: move the motor position -> wheel position conversions to a dedicated kinematics class

    // not sure if there's a unit-safe way to do this
    auto wheel_circumference = wheel_diameter.in(au::meters) * M_PI;
    auto linear_distance = 
        revolutions.in(au::revolutions) 
        * wheel_circumference 
        * gear_ratio;
    
    return au::meters(linear_distance);
}

au::Quantity<au::MetersPerSecond, double> Chassis::rpm_to_velocity(au::Quantity<au::Rpm, double> rpm) const {
    // TODO: move the motor velocity -> wheel velocity conversions to a dedicated kinematics class
    
    // not sure if there's a unit-safe way to do this
    auto wheel_circumference = wheel_diameter.in(au::meters) * M_PI;
    auto linear_velocity = 
        (rpm.in(au::rps) 
        * wheel_circumference 
        * gear_ratio);
    
    return au::meters_per_second(linear_velocity);
}

au::Quantity<au::Meters, double> Chassis::left_motors_displacement() const {
    auto revolutions = left_motors.get_position();
    auto displacement = revolutions_to_displacement(revolutions);
    
    return displacement;
}

au::Quantity<au::Meters, double> Chassis::right_motors_displacement() const {
    auto revolutions = right_motors.get_position();
    auto displacement = revolutions_to_displacement(revolutions);

    return displacement;
}

au::Quantity<au::Meters, double> Chassis::forward_motor_displacement() const {
    return (left_motors_displacement() + right_motors_displacement()) / 2.0;
}

au::Quantity<au::MetersPerSecond, double> Chassis::left_motors_velocity() const {
    auto rpm = left_motors.get_velocity();
    auto velocity = rpm_to_velocity(rpm);
    
    return velocity;
}

au::Quantity<au::MetersPerSecond, double> Chassis::right_motors_velocity() const {
    auto rpm = right_motors.get_velocity();
    auto velocity = 
        rpm_to_velocity(rpm);

    return velocity;
}

au::Quantity<au::MetersPerSecond, double> Chassis::forward_motor_velocity() const {
    return (left_motors_velocity() + left_motors_velocity()) / 2.0;
}

}