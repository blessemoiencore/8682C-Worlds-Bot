#include "main.h"
using namespace pros;


// void turnToHeadingWithGainScheduler( float targetHeading) {
//     // Define default parameters
//     lemlib::TurnToHeadingParams params = {
//         .maxSpeed = 127,  // Maximum motor speed
//     };

//     // Adjust PID constants and other parameters based on target heading
//     if (targetHeading == 90) {
//         params.kP = 12;
//         params.kI = 0.4;
//         params.kD = 0.02;
//         params.minSpeed = 15;
//     } else if (targetHeading == 180) {
//         params.kP = 10;
//         params.kI = 0.5;
//         params.kD = 0.03;
//         params.minSpeed = 20;
//     } else {
//         // Default PID constants for other angles
//         params.kP = 8;
//         params.kI = 0.3;
//         params.kD = 0.01;
//         params.minSpeed = 10;
//     }

//     // Call turnToHeading with the adjusted parameters
//     chassis.turnToHeading(targetHeading, 2000 /* timeout in ms */, params, false /* synchronous */);
// }
