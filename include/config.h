#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
using namespace pros;



extern Imu imu1;
extern MotorGroup left_motors;
extern MotorGroup right_motors;
extern lemlib::Drivetrain drivetrain;
extern Rotation horizontal_rotation;
extern Rotation vertical_rotation;
//extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::Chassis chassis;
extern Motor intake;
extern Motor conveyor;
extern Motor lift;
extern Controller remote;
extern Rotation lb_rotation;
extern adi::Pneumatics grab;
extern adi::Pneumatics intakeLift;
extern adi::Pneumatics doinker;
extern adi::Button limitSwitch;
extern float get_expo_value(float joystick_value, float deadband);

//extern void move_lift(float angle);
//extern void lift_control(float angle);

