#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-4,-5,-2});
pros::MotorGroup right_mg({11,12,13});
pros::Motor intake1({3});
pros::Motor intake2({10});
pros::MotorGroup intake({3,10});
pros::Imu inertial(12);

