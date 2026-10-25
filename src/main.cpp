#include "main.h"
#include "au/au.hpp"
#include "dlib/controllers/error_derivative_settler.hpp"
#include "dlib/controllers/error_time_settler.hpp"
#include "dlib/controllers/feedforward.hpp"
#include "dlib/controllers/pid.hpp"
#include "dlib/dlib.hpp"
#include "dlib/hardware/chassis.hpp"
#include "dlib/kinematics/odometry.hpp"
#include "dlib/trajectories/trapezoid_profile.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include <algorithm>
#include <initializer_list>
#include <chrono>
#include <thread>
#include <cmath>
using namespace au;
// noob vs pros vs hacker
// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Program motor placement for drive wheelss
pros::Motor leftFrontDrive(-11,pros::v5::MotorGears::blue);
pros::Motor leftMiddleDrive(-12,pros::v5::MotorGears::blue);
pros::Motor leftBackDrive(-14, pros::v5::MotorGears::blue);

pros::Motor rightFrontDrive(20,pros::v5::MotorGears::blue);
pros::Motor rightMiddleDrive(19,pros::v5::MotorGears::blue);
pros::Motor rightBackDrive(17, pros::v5::MotorGears::blue);

pros::Motor cascadeLeft(3, pros::v5::MotorGears::green);
pros::Motor cascadeRight(-9, pros::v5::MotorGears::green);

// pros::Motor loadingMini(-18); // half motor

pros::adi::DigitalOut pistonClaw('a', LOW);

pros::MotorGroup leftDrive({-11,-12,-14});
pros::MotorGroup rightDrive({20,19,17});

pros::MotorGroup cascade({3, -9});



class Robot {
public:
	dlib::Chassis chassis;
	dlib::Imu imu;

	dlib::Odometry odom;
	std::unique_ptr<pros::Task> odom_updater;

	dlib::Pid<Meters> move_pid;
	dlib::ErrorDerivativeSettler<Meters> move_settler;

	dlib::Pid<Degrees> turn_pid;
	dlib::ErrorDerivativeSettler<Degrees> turn_settler;

	dlib::Feedforward<Meters> move_feedforward;
	dlib::Pid<Meters> linear_feedforward_pid;

	Robot(
		dlib::ChassisConfig chassis_config, 
		dlib::ImuConfig imu_config,
		dlib::PidConfig move_pid_config,
		dlib::ErrorDerivativeSettler<Meters> move_pid_settler,
		dlib::PidConfig turn_pid_config,
		dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler,
		dlib::FeedforwardGains move_feedforward_gains,
		dlib::PidConfig linear_feedforward_pid_config
	) : 
		chassis(chassis_config), 
		imu(imu_config),
		move_pid(move_pid_config),
		move_settler(move_pid_settler),
		turn_pid(turn_pid_config),
		turn_settler(turn_pid_settler),
		move_feedforward(move_feedforward_gains),
		linear_feedforward_pid(linear_feedforward_pid_config),
		odom() {

	}

	void calibrate_imu() {
		imu.initialize();
	}

	// Odom task
	void start_odom() {
		chassis.initialize();

		odom_updater = std::make_unique<pros::Task>([this]() {
			while (true) {
				odom.update(
					chassis.left_motors_displacement(), 
					chassis.right_motors_displacement(), 
					ZERO,
					imu.get_rotation()
				);

				pros::delay(20);
			}
		});
	}

	// Use PID to do a relative movement
	void move_with_pid(Quantity<Meters, double> displacement, double max_voltage = 12000){
		auto start_displacement = chassis.forward_motor_displacement();
		auto target_displacement = dlib::relative_target(start_displacement, displacement);
		
		move_pid.reset();
		move_settler.reset();

		while (!move_settler.is_settled(move_pid.get_error(), move_pid.get_derivative())) {
			auto error = dlib::linear_error(target_displacement, chassis.forward_motor_displacement());
			pros::lcd::print(3,"error: %f", error.in(inches));
			
			auto voltage = move_pid.update(error, milli(seconds)(20));

			if(std::abs(voltage.in(milli(volts))) > max_voltage){
				if(voltage.in(milli(volts)) > 0){
					voltage = milli(volts)(max_voltage);
				}
				else{
					voltage = milli(volts)(-max_voltage);
				}
			}
			
			chassis.move_voltage(voltage);

			pros::delay(20);
		}
		pros::lcd::print(4,"settled");
		chassis.brake();
	}

	void move_with_pid(double displacement, double max_voltage = 12000) {
		move_with_pid(inches(displacement),max_voltage);
	}

	void move_boomerang(double x, double y, double max_move, double kp_turn, bool reverse = true, double kp_move=175){
		while(true){
			auto point = dlib::Vector2d(inches(x),inches(y));
			auto displacement = odom.displacement_to(point,reverse);
			auto heading = odom.angle_to(point,reverse);
			auto angle = dlib::angular_error(heading, imu.get_rotation());

			auto power = displacement.in(meters) * kp_move;
			auto turn = angle.in(degrees) * kp_turn;

			auto left = power + turn;
			auto right = power - turn;

			if(left > max_move){
				left = max_move;
			}
			if(right > max_move){
				right = max_move;
			}

			chassis.left_motors.move(left);
			chassis.right_motors.move(right);

			if(std::abs(displacement.in(inches)) < 1.5){
				break;
			}
			pros::delay(20);
		}
		chassis.brake();
		// :)
	}

	void turn_with_pid(Quantity<Degrees, double> heading) {
		auto target_heaidng = heading;

		turn_pid.reset();
		turn_settler.reset();

		while (!turn_settler.is_settled(turn_pid.get_error(), turn_pid.get_derivative())) {
			auto error = dlib::angular_error(heading, imu.get_rotation());
			
			auto voltage = turn_pid.update(error, milli(seconds)(20));
			
			chassis.turn_voltage(voltage);

			pros::delay(20);
		}
		chassis.brake();
	}

	void turn_with_pid(double heading,bool reverse=false) {
		turn_with_pid(degrees(heading));
	}

	void turn_to_point(dlib::Vector2d point,bool reverse=false) {
		auto angle = odom.angle_to(point,reverse);
		turn_with_pid(angle);
	}

	void turn_to_point(double x, double y, bool reverse=false) {
		turn_to_point({inches(x), inches(y)},reverse);
	}

	void move_to_point(dlib::Vector2d point, bool reverse=false, double max_voltage = 12000) {
		turn_to_point(point,reverse);

		auto displacement = odom.displacement_to(point,reverse);
		// move_with_pid(displacement,max_voltage);
		move_with_ffwd(displacement.in(inches),max_voltage);
	}

	void move_to_point(double x, double y, bool reverse=false,double max_voltage = 12000) {
		move_to_point({inches(x), inches(y)},reverse,max_voltage);
	}

	void move_with_ffwd(double displacement, double max_velocity = 12000){
		auto start_displacement = chassis.forward_motor_displacement();
		dlib::TrapezoidProfile<Meters> profile = dlib::TrapezoidProfile<Meters>::from_constraints(
			inches(displacement),
			meters_per_second(max_velocity/39.37),
			meters_per_second_squared(2)
		);

		linear_feedforward_pid.reset();
		move_settler.reset();

		auto elapsed_time = 0;
		auto current_time = pros::millis();
		auto start_time = pros::millis();

		//auto total_time = profile.get_time();

		while (true) {
			current_time = pros::millis();
			elapsed_time = current_time - start_time;

			auto setpoint = 
			profile.calculate(milli(seconds)(elapsed_time));

			auto current_position = chassis.forward_motor_displacement();
			auto target_position = 
			dlib::relative_target(start_displacement, setpoint.position);

			auto error = dlib::linear_error(target_position, current_position);

			auto pid_voltage = 
			linear_feedforward_pid.update(error, milli(seconds)(10));
			auto ff_voltage = 
			move_feedforward.calculate(setpoint.velocity, 
				setpoint.acceleration);
			
			if(profile.stage(milli(seconds)(elapsed_time)) == dlib::TrapezoidProfileStage::Decelerating){
				chassis.move_voltage(volts(0));
			}

			if(profile.stage(milli(seconds)(elapsed_time)) == 
			dlib::TrapezoidProfileStage::Done){
				break;
			}	

			//std::cout << "voltage: " << ff_voltage << "\n";

			std::cout << elapsed_time << "," << setpoint.velocity.in(au::meters_per_second)<< "," << chassis.forward_motor_velocity().in(au::meters_per_second) << "\n";

			chassis.move_voltage(ff_voltage + pid_voltage);
			
			pros::delay(10);
		}
		chassis.move_voltage(volts(0));
}
};

// Create a config for everything used in the Robot class
dlib::ChassisConfig chassis_config = dlib::ChassisConfig::from_drive_rpm(
	{{-11, -12, -14}},	// left motors
	{{20, 19, 17}},	// right motors



	pros::MotorGearset::blue,
	rpm(400),	// the drivebase rpm
	inches(2.75)	// the drivebase wheel diameter
);

dlib::RotationConfig rotation_config = dlib::RotationConfig(
	9, //rot sensor
	inches(0.3183098862),
	1
);


dlib::ImuConfig imu_config {
	17,	// imu port // 11
	1.00	// optional imu scaling constant
};

dlib::PidConfig move_pid_config {
	{
		135, // kp, proportional gain
		0, 	// ki, integral gain
		10	// kd, derivative gain
	}
};

// dlib::PidConfig move_pid_config {
// 	{
// 		135, 	// kp, proportional gain
// 		0, 	// ki, integral gain
// 		10	// kd, derivative gain
// 	}
// };

dlib::FeedforwardGains move_feedforward_gains {
	1.094123824483574, // ks, static friction
	8.05189133025693, // kv, scalable velocity
	1.1206435619911086 // ka, acceleration
};

dlib::PidConfig linear_feedforward_pid_config {
	{
		50, // kp, porportional gain
		0, 	// ki, integral gain
		0	// kd, derivative gain
	}
};

dlib::ErrorDerivativeSettler<Meters> move_pid_settler {
	inches(0.8),		// error threshold, the maximum error the pid can settle at
	meters_per_second(0.1) // derivative threshold, the maximum instantaneous error over time the pid can settle at
};

dlib::PidConfig turn_pid_gains {
	{
		45, 	// kp, porportional gain
		0, 	// ki, integral gain
		3.5 	// kd, derivative gain
	}
};

dlib::ErrorDerivativeSettler<Degrees> turn_pid_settler {
	degrees(2),		// error threshold, the maximum error the pid can settle at
	degrees_per_second(20)	// derivative threshold, the maximum instantaneous error over time the pid can settle at
};

Robot robot = Robot(
	chassis_config,
	imu_config,
	move_pid_config,
	move_pid_settler,
	turn_pid_gains,
	turn_pid_settler,
	move_feedforward_gains,
	linear_feedforward_pid_config
);

void on_center_button() {}

void screen()  {
    while (true)  {
        pros::delay(10);
    }
}

void initialize() {
	robot.chassis.initialize();
	robot.calibrate_imu();
	robot.start_odom();
	pros::lcd::initialize(); // initialize brain screen
	// pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
	pros::Task screenTask(screen); // create a task to print the position to the screen
    pros::lcd::initialize(); // initialize brain screen
    pros::Task screen_task([&]() {
        while (true) {
            pros::delay(20);
			dlib::Pose2d pose = robot.odom.get_position();
			pros::lcd::print(0,"x: %f", pose.x.in(inches));
			pros::lcd::print(1,"y: %f", pose.y.in(inches));
			pros::lcd::print(2,"theta: %f", pose.theta.in(degrees));
        }
    });



	auto profile = 
	dlib::TrapezoidProfile<Meters>::from_constraints(
		centi(meters)(60),
		meters_per_second(1.5),
		meters_per_second_squared(3)
	);

	profile.calculate(seconds(0));
	
	
}

void disabled() {}

void competition_initialize() {

}


void manualMove(int volts){
	leftDrive.move(volts);
	rightDrive.move(volts);
}

// void manualBrakeStop(){
// 	brakeStop(leftSide);
// 	brakeStop(rightSide);
// }



// BRAKE MODES MADE BY ADYAN AND DAN USE LATER
void brakeStop(pros::Motor& motor){
	motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	motor.brake();
}
void brakeStop(pros::MotorGroup& motor){
	motor.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
	motor.brake();
}
void brakeCoast(pros::Motor& motor){
	motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	motor.brake();
}
void brakeCoast(pros::MotorGroup& motor){
	motor.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	motor.brake();
}






void move_to_point_adjusted(double x, double y, double starting_x, double starting_y, bool reverse = false, double max_volts = 12000){
	robot.move_to_point(x + starting_x,y + starting_y, reverse, max_volts);
}








// void leftSideAuto(){
// 	robot.start_odom();
// 	pistonHood.set_value(HIGH);
// 	intakeFront.move(127);
// 	loadingMini.move(127);
// 	scoringMini.move(-127);
// 	// pros::Task intakeTask([](){
// 	// 	intakeFront.move(127);
// 	// 	loadingMini.move(127);
// 	// });

// 	//first get balls
// 	// robot.move_boomerang(-18,8,5,1.5, true, 175);
// 	// robot.move_with_ffwd(-4, 10);

// 	robot.move_to_point(-18,9, true, 25);
// 	pros::delay(75);
	
// 	// intakeTask.remove();
// 	// brakeStop(loadingMini);
// 	// robot.turn_with_pid(-135);
// 	// pros::delay(20);
// 	// robot.move_to_point(-24.6,0.325);
// 	// robot.move_to_point(-27.9,-0.95);

// 	//go to score them
// 	// robot.move_to_point(-20.5,-0.5, false, 30);
// 	robot.move_to_point(-20.75,-0.5, false, 30); //gay robot
// 	pros::delay(10);

// 	robot.turn_with_pid(-136.5);

// 	// robot.move_with_pid(2);
// 	robot.move_with_ffwd(2.5);
// 	pros::delay(25);

// 	brakeStop(intakeFront);
// 	brakeStop(loadingMini);
// 	brakeStop(scoringMini);
// 	intakeFront.move(-127);
// 	loadingMini.move(-127);
// 	scoringMini.move(-127);
// 	pros::delay(150);
// 	brakeStop(intakeFront);
// 	brakeStop(loadingMini);
// 	brakeStop(scoringMini);
// 	pros::delay(25);
// 	intakeFront.move(127);
// 	loadingMini.move(64);
// 	scoringMini.move(64); // low score
// 	pistonHood.set_value(LOW);

// 	pros::delay(645);

// 	dlib::Pose2d currentpose = robot.odom.get_position();
// 	double current_x = currentpose.x.in(inches);
// 	double current_y = currentpose.y.in(inches);
// 	// move_to_point_adjusted(12, 0, current_x, current_y, true, 25

// 	// brakeStop(scoringMini);
// 	// feather_task.remove();
// 	pistonHood.set_value(HIGH);
// 	brakeStop(scoringMini);
// 	scoringMini.move(-127);
// 	robot.move_with_ffwd(-7, 30);
// 	brakeStop(loadingMini);
// 	loadingMini.move(-127);


// 	//run away to matchloader

// 	// robot.move_to_point(-1.5, 22.5 , false, 25);
// 	move_to_point_adjusted(23.5, 24, current_x, current_y,true, 22.5); //+3, -3


// 	//now repeat
// 	pros::delay(10);//was `125
// 	robot.turn_with_pid(-180);
// 	pros::delay(10);

// 	// intakeFront.move(127);
// 	brakeStop(loadingMini);
// 	loadingMini.move(127);
// 	// scoringMini.move(-127);
// 	pistonMatchLoad.set_value(HIGH);
// 	// pistonIntake.set_value(HIGH);
// 	pistonHood.set_value(HIGH);
// 	pros::delay(125);
// 	//go in

// 	robot.move_with_ffwd(-10.75, 20);

// 	// brakeStop(leftSide);
// 	// brakeStop(rightSide);

// 	pros::delay(345);

// 	dlib::Pose2d currentpose2 = robot.odom.get_position();
// 	double current_x2 = currentpose2.x.in(inches);
// 	double current_y2 = currentpose2.y.in(inches);


// 	//leave
// 	robot.move_with_ffwd(5, 20);

// 	pistonMatchLoad.set_value(LOW);
// 	// robot.move_with_ffwd(13.25, 10);
// 	move_to_point_adjusted(-20, -3, current_x2, current_y2,false, 35); //+3, -3

// 	pistonHood.set_value(LOW);
// 	pros::delay(10);
// 	brakeStop(intakeFront);
// 	brakeStop(loadingMini);
// 	intakeFront.move(-127);
// 	loadingMini.move(-127);
// 	pros::delay(75);
// 	brakeStop(intakeFront);
// 	brakeStop(loadingMini);
// 	pros::delay(10);
// 	intakeFront.move(127);
// 	loadingMini.move(127);

// 	// loadingMini.move(127);
// 	// scoringMini.move(-127);
// 	pros::delay(925);
// 	brakeStop(loadingMini);
// 	brakeStop(scoringMini);


	

// 	// dlib::Pose2d currentpose3 = robot.odom.get_position();
// 	// double current_x3 = currentpose3.x.in(inches);
// 	// double current_y3 = currentpose3.y.in(inches);
// 	// move_to_point_adjusted(5, 0, current_x3, current_y3, true, 25);
// 	robot.move_with_ffwd(-3, 25);
// 	robot.turn_with_pid(270);
// 	robot.move_with_ffwd(-7.8, 22);
// 	pistonWing.set_value(HIGH);
// 	robot.turn_with_pid(0);
// 	pistonWing.set_value(LOW);
// 	robot.move_with_ffwd(-16.8, 21);
// 	// robot.move_with_ffwd(-12);
// }



// }
void record_map_voltages(){
	double current_voltage = 0;

	double start_time = pros::millis();

	while (current_voltage <= 12000){
		double current_time= pros::millis();
		double elapsed_time = current_time - start_time;

		//current_voltage = elapsed_time/1000 * 2;
		current_voltage = 7000;

		double current_position = robot.chassis.forward_motor_displacement().in(meters);
		double velocity = robot.chassis.forward_motor_velocity().in(au::meters_per_second);

		robot.chassis.move_voltage(volts(current_voltage));

		std::cout << elapsed_time/1000.0 << "," << velocity << std::endl;

		pros::delay(10);
	}

	robot.chassis.brake();
}

void autonomous() {
	robot.start_odom();

	// leftSideAuto();

	dlib::Pose2d pose = robot.odom.get_position();
	pros::lcd::print(0,"x: %f", pose.x.in(inches));
	pros::lcd::print(1,"y: %f", pose.y.in(inches));
	pros::lcd::print(2,"theta: %f", pose.theta.in(degrees));
}




double start = pros::millis(); 

void opcontrol() {

	bool clawToggle = true;

	leftDrive.set_gearing(pros::MotorGears::blue);
	rightDrive.set_gearing(pros::MotorGears::blue);
	leftDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	rightDrive.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);


	while(true){

		double elapsed = pros::millis();

		// get power and turn
		double power = master.get_analog(ANALOG_LEFT_Y);
		double turn = master.get_analog(ANALOG_RIGHT_X) * -1;
		// ^weird way of inverting the turn value because flipping symbol below wont work for some reason
		
		leftDrive.move(power + turn);
		rightDrive.move(power - turn);

		// piston claw toggle
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			if (clawToggle) {
				pistonClaw.set_value(HIGH);
			} else {
				pistonClaw.set_value(LOW);
			}
			
			clawToggle = !clawToggle;
			pros::delay(250);
		}

		// CASCADE
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){ // cascade up
			cascade.move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){ // cascade down
			cascade.move(-50);
		} else {
			brakeStop(cascade);
		}

		// arcade movement
		robot.chassis.arcade(power, turn);
		dlib::Pose2d pose = robot.odom.get_position();
		pros::lcd::print(0,"x: %f", pose.x.in(inches));
		pros::lcd::print(1,"y: %f", pose.y.in(inches));
		pros::lcd::print(2,"theta: %f", pose.theta.in(degrees));
		// printData(elapsed);
		pros::delay(20);

	}
}