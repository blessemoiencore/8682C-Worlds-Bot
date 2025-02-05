#include "main.h"
#include "autons.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "cmath"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pid.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include <cmath>
#include <iostream>
#include "gainschedule.h"
#include "pros/rtos.hpp"


Controller remote(pros::E_CONTROLLER_MASTER);
bool auto_started = false;

void lift_control(float angle) {
    double tolerance = 0;
    while ((fabs(angle - lb_rotation.get_position()) > tolerance)) {
        float kp = 0.0145;
        float error = angle - lb_rotation.get_position();
        float voltage = kp * error;
        lift.move(voltage);
        pros::delay(20);
    }

    lift.set_brake_mode(MotorBrake::hold);
    lift.brake();
    pros::delay(10);
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
		
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 *
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "8682C");
	chassis.calibrate();
	//lady brown task
	
	 pros::Task odom_task([]{
		while (true) {
		 pros::lcd::print(2, "vertical sensor: %f", ((vertical_rotation.get_position() * 3.14159265  * 2.75)) / 36000 );
		 pros::delay(10);
		}

	 });
	 

	pros::lcd::register_btn1_cb(on_center_button);

	

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
auto_started = true;
red_goal_rush();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {



	while (true) {

			pros::lcd::print(3, "angular: %f", imu1.get_heading());

			pros::lcd::print(4, "pid gains: %i", vertical_rotation.get_position()); // what will this do??? 

		
		if(remote.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			doinker.toggle();
		}

		if(remote.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			grab.toggle();
		}

		if(remote.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			intakeLift.toggle();
		}

		if (remote.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			//lift.move_absolute(-1250, 170); //scoring pose
			lift.move_absolute(-230, 170);
		}
	
		if(remote.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.move(127);
			conveyor.move(127);
		}
		else {
			intake.brake();
			conveyor.brake();
					}

		//arcade
		int dir = remote.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = remote.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_motors.move(dir + turn);                      // Sets left motor voltage
		right_motors.move(dir - turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}