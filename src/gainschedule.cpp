#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include"lemlib/api.hpp"

void turnToHeading_GS(float theta, int timeout, lemlib::TurnToHeadingParams params = {} ) {
    if (theta == 90) {
        chassis.angularPID.setGains(20, 20, 50);
        //angular_controller.kP = 20
    }
    else if (theta == 180) {
        chassis.angularPID.setGains(1.85, 0.3, 12.5);
    }
    chassis.turnToHeading(theta,  timeout, params = {});
}
