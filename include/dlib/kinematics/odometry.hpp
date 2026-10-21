#pragma once
#include "au/au.hpp"
#include "pros/rtos.hpp"

namespace dlib {

/**
 * @brief A point in 2d space
 * 
 */
struct Vector2d {
    /** The x component of the point */
    au::Quantity<au::Meters, double> x;

    /** The y component of the point */
    au::Quantity<au::Meters, double> y;

    Vector2d(
        au::Quantity<au::Meters, double> x, 
        au::Quantity<au::Meters, double> y
    );
};

/**
 * @brief A pose in 2d space
 * 
 */
struct Pose2d {
    /** The x component of the pose */
    au::Quantity<au::Meters, double> x;

    /** The y component of the pose */
    au::Quantity<au::Meters, double> y;

    /** The heading of the pose */
    au::Quantity<au::Degrees, double> theta;

    Pose2d(
        au::Quantity<au::Meters, double> x, 
        au::Quantity<au::Meters, double> y,
        au::Quantity<au::Degrees, double> theta
    );
};

class Odometry {
public:
    Odometry(au::Quantity<au::Meters, double> horizontal_wheel_offset = au::ZERO);

    /**
     * @brief Update the odometry tracking state
     * 
     * @param left_displacement the linear displacement of the left side of the Chassis
     * @param right_displacement the linear displacement of the right side of the Chassis
     * @param horizontal_displacement the linear displacement from any horizontal tracking
     * @param heading the current heading
     */
    void update(
        au::Quantity<au::Meters, double> left_displacement,
        au::Quantity<au::Meters, double> right_displacement,
        au::Quantity<au::Meters, double> horizontal_displacement,
        au::Quantity<au::Degrees, double> heading
    );

    /**
     * @brief Get the current position of the robot
     * 
     * @return Pose2d 
     */
    Pose2d get_position() const;

    /**
     * @brief Set the current position of the robot
     * 
     * @param pose 
     */
    void set_position(Pose2d pose);

    /**
     * @brief Calculate the displacement from the current position of the robot to a point
     * 
     * @param point the end point
     * @param reverse if the robot will move in reverse
     * @return the displacement to the point
     */
    au::Quantity<au::Meters, double> displacement_to(Vector2d point, bool reverse = false) const;

    /**
     * @brief Calculate the angle from the current position of the robot to a point
     * 
     * @param point the end point
     * @param reverse if the robot will move in reverse
     * @return the angle to the point
     */
    au::Quantity<au::Degrees, double> angle_to(Vector2d point, bool reverse = false) const;

protected:
    Pose2d m_position = Pose2d(au::ZERO, au::ZERO, au::ZERO);
    
    au::Quantity<au::Meters, double> m_prev_forward = au::ZERO;
    au::Quantity<au::Meters, double> m_prev_horizontal = au::ZERO;
    au::Quantity<au::Radians, double> m_prev_theta = au::ZERO;

    const au::Quantity<au::Meters, double> m_horizontal_wheel_offset = au::ZERO;

    pros::Mutex m_mutex{};
};

}