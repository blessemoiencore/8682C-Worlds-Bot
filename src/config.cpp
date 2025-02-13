
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pid.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <chrono>
#include <cstddef>
#include "cmath"
#include "config.h"
using namespace pros;


//motor groups
MotorGroup left_motors({-3,-2,10,}, MotorGearset::blue);
MotorGroup right_motors({13,-12,19}, MotorGearset::blue);

//drivetrain configuration
lemlib::Drivetrain drivetrain(&left_motors,
                      &right_motors,
                      12.25,
                      lemlib::Omniwheel::NEW_325,
                      450,
                      8 // mess with this
);

//sensors
Imu imu1(5);//port
Rotation lb_rotation(1); 
adi::Button limitSwitch('D');

//tracking wheel configuration
//Rotation horizontal_rotation(1);
Rotation vertical_rotation(4);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation, lemlib::Omniwheel::NEW_275,   .75); //change 0.85
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu1 // inertial sensor
);

//PID configurations
lemlib::ControllerSettings lateral_controller(3.75, // proportional gain (kP) /4 works
                                              0.2, // integral gain (kI)
                                              8, // derivative gain (kD) /8 works
                                              3, // anti windup
                                              0.5, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              1.5, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew) //was 20
);



//angular PID controller
lemlib::ControllerSettings angular_controller(1.85, // proportional gain (kP)
                                              0.3, // integral gain (kI)
                                              12.5, // derivative gain (kD)
                                              3, // anti windup
                                              0.5, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              1.5, // large error range, in degrees
                                              600, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

//creating the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors);


//controller


//pneumatics
adi::Pneumatics grab('A', false);
adi::Pneumatics intakeLift('H',false);
adi::Pneumatics doinker('G', false);

//motors
Motor intake(16, MotorGears::green);
Motor conveyor(20, MotorGears::blue);
Motor lift(11,MotorGears::green);

//functions
template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

float get_expo_value(float joystick_value, float deadband) {
    if (std::fabs(joystick_value) < deadband) {
    return 0;
  }
    else return sgn(joystick_value) *  (pow(sgn(joystick_value) * joystick_value, 1.9) ) / 63.095734448;
}



