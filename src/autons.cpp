#include "autons.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

void example_auton() {
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    //chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}

void PID_tuning() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 100000);
}

void test_drive() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(1, 2, 90, 4000, {.lead = 0.4}); // mess with drift

    /** 
    * @brief motion chaining example
    * sets minimum speed and an early stop distance
    * if within that range exit movement
    */
    chassis.moveToPose(20, 
                       40, 
                       100, 
                       1000, 
                       {.minSpeed = 60, .earlyExitRange =8}
                       );

    chassis.moveToPose(38, 45, 120, 1000);
    grab.extend();
    grab.retract();
    grab.toggle();




}

void skills() {/*
conveyor.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
chassis.setPose(0,0,0);
chassis.turnToHeading((90), 10000);
*/

//scoring the preload onto the alliance stake
//chassis.angularPID.setGains(2.25, 0, 12.5); home pid constants
chassis.setPose(-60,0,90);
intakeLift.set_value(true);
conveyor.move(127);
delay(500);
conveyor.brake();


//grabbing first mobile goal
chassis.moveToPoint(-48, 0, 900,{.forwards = true, .maxSpeed = 60});
chassis.waitUntilDone();
chassis.turnToHeading(0, 1100);
chassis.moveToPoint(-48, -22, 1000,{.forwards = false, .maxSpeed =50 });
delay(570);
grab.extend();


//going for the first and second rings
intake.move(127);
conveyor.move(127);
chassis.turnToHeading(90, 1100);
delay(200);
lift.move_absolute(-500, 70);
chassis.moveToPoint(-21, -22, 1000); //-21, -27
chassis.moveToPose(17, -44,110, 3000, {.forwards = true, .maxSpeed = 127, .minSpeed = 30, .earlyExitRange = 5});
conveyor.move(127);
delay(1000);

//wallstake ring
chassis.swingToHeading(250, lemlib::DriveSide::LEFT, 1500, {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 4});
chassis.waitUntilDone();
chassis.moveToPose(-2, -100, 180, 2800, {.lead = 0.67, .maxSpeed = 60});
chassis.waitUntil(14);
chassis.cancelMotion();
delay(1500);

//rush!!!!
chassis.swingToHeading(290, lemlib::DriveSide::LEFT, 1500, {.maxSpeed = 90});
chassis.waitUntilDone();
conveyor.brake();
conveyor.move(120);
chassis.moveToPose(-37, -48, 270, 1500, {.maxSpeed = 50, . minSpeed = 20, .earlyExitRange = 4});
chassis.moveToPoint(-43, -48, 1000, {.maxSpeed = 40});
delay(500);
chassis.moveToPoint(-54, -48, 900, {.maxSpeed = 40});
chassis.waitUntilDone();
chassis.moveToPoint(-70, -48, 500, {.maxSpeed = 60});
chassis.waitUntilDone();
chassis.setPose(-60,-49.5,270);




/*
chassis.turnToHeading(135, 700);
chassis.moveToPoint(-52, -55, 1000);
conveyor.brake();
chassis.moveToPose(float x, float y, float theta, int timeout)
*/

//reset pose here
//then turn to last ring and keep in conveyor
//chassis.moveToPose(-47,18,0, 2000, {.lead = 0.3});



}

void red_goal_rush() {
    left_motors.set_brake_mode(MotorBrake::hold);
    right_motors.set_brake_mode(MotorBrake::hold);
    chassis.setPose(-52,-37,90);
    chassis.moveToPose(-10, -38, 115, 1000, {.lead = 0.4, .maxSpeed = 127, .minSpeed = 127, .earlyExitRange = 2});
    /*
    chassis.waitUntil(40);
    chassis.turnToHeading(115, 500);
    chassis.waitUntilDone();
    doinker.extend();
    left_motors.brake();
    right_motors.brake();
    //chassis.moveToPoint(-31, -38, 800, {.forwards = false, .maxSpeed = 127, . minSpeed= 127, .earlyExitRange = 2});
    */
}

void blue_goal_rush() {
    left_motors.set_brake_mode(MotorBrake::hold);
    right_motors.set_brake_mode(MotorBrake::hold);
    chassis.setPose(51.5, -60, 270);

    ///goal rush
    chassis.moveToPoint(14, -60, 900, {.minSpeed = 80});
    chassis.waitUntil(44);
    chassis.turnToHeading(300, 300);
    left_motors.brake();
    right_motors.brake();
    doinker.extend();
    chassis.turnToHeading(205, 700);
    delay(300);
    chassis.moveToPose(60, -27,240, 1400, {.forwards = false, .maxSpeed = 80,});
    delay(900);
    doinker.retract();

    //second goal
    chassis.swingToHeading(145, lemlib::DriveSide::LEFT, 1000, {.maxSpeed = 80});
    chassis.moveToPoint(25, -26, 1000,{.forwards = false}); //25, -26
    delay(1000);
    grab.extend();
    delay(200);

    //preload
    conveyor.move(127);
    delay(500);
    grab.retract();

    //ring 1
    chassis.swingToHeading(70, lemlib::DriveSide::LEFT, 700);
    conveyor.brake();
    delay(100);
    intakeLift.extend();
    intake.move(100);
    conveyor.move(127);
    chassis.moveToPoint(55, -29, 800, {.maxSpeed = 90});
    delay(750);
    conveyor.brake();

    //pile ring
    chassis.turnToHeading(355, 800);
    intakeLift.retract();
    chassis.moveToPoint(52 , -4.5, 800);
    chassis.waitUntilDone();
    intakeLift.extend();
    delay(500);

    //second mogo
    chassis.moveToPoint(52, -18, 700, {.forwards = false});
    chassis.turnToHeading(45, 800);
    chassis.moveToPose(3,-42, 90, 1400, {.forwards = false, .lead = 0.7 });
    chassis.waitUntilDone();

    //two rings
    grab.extend();
    conveyor.move(127);
    intake.brake();

    //ladder
    chassis.turnToHeading(135, 800);
    //chassis.moveToPoint(5, -28, 1000, {.maxSpeed = 120});

}

void blue_goal_rushV2() {
    chassis.setPose(43,-38,242.7);
    left_motors.set_brake_mode(MotorBrake::hold);
    right_motors.set_brake_mode(MotorBrake::hold);

    doinker.extend();
    intakeLift.extend();
    intake.move(127);
    chassis.moveToPoint(17, -50, 1000, {.minSpeed = 80});
    left_motors.brake();
    right_motors.brake();
    delay(1000);
    doinker.retract();
}


// simple auton for quals, 
//3 ring wallstake auton
void red_pos_quals() {
    chassis.setPose(-53,-16.5, 90);

    //pile ring
    chassis.turnToHeading(15 , 1000);
    chassis.moveToPoint(-50, -7, 800, {.maxSpeed = 70});
    chassis.waitUntilDone();
    intakeLift.extend();
    intake.move(127);
    delay(700);

    //mogo
    chassis.turnToHeading(305, 1000);
    chassis.moveToPoint(-27, -22, 1500, {.forwards = false, .maxSpeed = 80});
    delay(1500);
    grab.extend();
    delay(400);
    conveyor.move(127);
    delay(1000);

    //ring 2
    chassis.turnToHeading(170, 1800);
    chassis.moveToPoint(-27, -43,1900);
    delay(2000);
    grab.retract();
   
    //wallstake
    chassis.turnToHeading(90, 1000);
    intakeLift.retract();
    delay(500);
    chassis.moveToPoint(-12, -46, 800);
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-13, -72, 1500);
    delay(1500);
    lift.move_absolute(-1700, 170);
    delay(1000);
    //chassis.moveToPoint(-13, -29, 1000, {.forwards = false, .maxSpeed = 70}); //40 rpm lb???
}