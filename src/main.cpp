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

void lift_control(void* param) {
	float angle = *(float*) param;
    double tolerance = 2;
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
	float angle = 20;
	 // lambda function, casts param to char* and prints as string
	auto my_task {[](void* param) {std::cout << "Function Parameters: " << (char*)param << std::endl;} };
	//Task lift_task(lift_control, (void*)&angle ,TASK_PRIORITY_MAX - 1, TASK_STACK_DEPTH_DEFAULT, "lift control");
	//Task lift_task_2(lift_control, (void*)"hello", 2, 1, "lift control2");
 	
	// Task taskOne (my_task,  //calling the task
	// 				 (void*)"hello, task!", //casts the string to type (void*) so the function can take it as param; can only be POINTER
	//				 TASK_PRIORITY_DEFAULT, //priority of the task, higher equals higher prio, typically default+1 or default -1 etc.
	//				 TASK_STACK_DEPTH_DEFAULT, //size of stack, or space; should usually be default
	//				 "My task name here"
	//   			)


	 pros::Task odom_task([]{
		//test this 
		while (auto_started) {
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
	blue_pos_ws();
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
	auto_started = false;


	while (true) {

			pros::lcd::print(3, "angular: %f", imu1.get_heading());

		if(limitSwitch.get_new_press()) {
			grab.extend();
		}
		
		if(remote.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			lift.move_absolute(-1700, 170); //40 rpm lb???
		}

		if(remote.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			lift.move_absolute(0, 170);
			intakeLift.retract();
		}
		
		
		if(remote.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
			doinker.toggle();
		}

		if(remote.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			grab.toggle();
		}

		if(remote.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			intakeLift.toggle();
			//lift_control((int*)20);
		}

		if (remote.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			lift.move_absolute(-230, 170);
		}
	
		if(remote.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.move(127);
			conveyor.move(127);
		}
		else if(remote.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intake.move(-127);
			conveyor.move(-127);
		}
		else {
			intake.brake();
			conveyor.brake();
					}

		//arcade      
		while (true) {
    	float dir = get_expo_value(remote.get_analog(ANALOG_LEFT_Y), 6.35);  // Apply exponential scaling to forward/backward
    	float turn = get_expo_value(remote.get_analog(ANALOG_RIGHT_X), 6.35); // Apply exponential scaling to turning

    	left_motors.move(dir + turn);   // Adjust left motor power
    	right_motors.move(dir - turn);  // Adjust right motor power

    	pros::delay(20);  // Small delay to prevent CPU overload
}                      // Run for 20 ms then update
	}
}