#include "main.h"
#include "robot-config.hpp"


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
/** */
double IntakeTempSpeed = 0;
static void MoveIntake() {
    int intakespeed;
    int intakemovespeed;
    if (master.get_digital_new_press(DIGITAL_L1)){
      intake.move(126);
    }
    else if (master.get_digital_new_press(DIGITAL_L2))
    {
      intake.move(-126);
    }
    else if (master.get_digital_new_press(DIGITAL_X))
    {
      intake.move(0);
    }
    else{
      intake.move(0);
    }
    pros::delay(20);
}
    /**if(master.get_digital_new_press(DIGITAL_L1)) {
        if (IntakeTempSpeed==1){
          IntakeTempSpeed = 0;
          intakespeed = 99;
        }      
        else if(IntakeTempSpeed == 0){
          IntakeTempSpeed = 1;
        }
        else{
          IntakeTempSpeed = 1;
        }

    if (IntakeTempSpeed == 1){
      intakespeed = 99;
    }
    else if (IntakeTempSpeed ==0){
      intakespeed = 35; 
    }
    else {
      intakespeed = 99;
    }
    
    if (master.get_digital_new_press(DIGITAL_L2)){
      intakemovespeed = 126;

    }
    if (master.get_digital_new_press(DIGITAL_L1)){
      intakemovespeed = -126;

    }
    if (master.get_digital_new_press(DIGITAL_X)){
      intakemovespeed = 0;

    }
    intake.move(intakemovespeed);
    pros::delay(20);
    }
}
*/


void turn(double heading) {
    double angle = fmod(heading - inertial.get_heading(), 360);
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
    double targetAngle = inertial.get_rotation() + angle;

    // constants to tune
    double kP = 100;
    double kI = 4;
    double kD = 90;
    double integral = 0;
    double lastError = targetAngle - inertial.get_rotation();

    int restedStates = 0;
    int stalledStates = 0;
    while (restedStates < 5 && stalledStates < 50) {
        double error = targetAngle - inertial.get_rotation();
        if (fabs(error) < 30) integral += error;
        if (fabs(error) < 0.1) integral = 0;

        if (fabs(error - lastError) < 0.005) stalledStates++;
        else stalledStates = 0;

        double out = kP * error + kI * integral + kD * (error - lastError);
        left_mg.move_voltage(out);
        right_mg.move_voltage(-out);

        lastError = error;

        pros::delay(10);

        if (fabs(targetAngle - inertial.get_rotation()) < 2) restedStates++;
        else restedStates = 0;
    }
    left_mg.move_voltage(0);
    right_mg.move_voltage(0);
}




void drivePID(){
	double kP=3.1;
double kI =1;
double kD =1;
int desiredposition;
	//Settings double kP double ki double kD
int error; //Sensorvalue
int prevError;  //Position 20 wiliseconds ago
int derivitave;
int totalerror;
bool enabledrivePID = true;
	while (enabledrivePID)
	{
		int leftdrivevalue = left_mg.get_position();
		int rightdrivevalue = right_mg.get_position();
		int avgposition = (left_mg.get_position()+right_mg.get_position())/2;
		error=desiredposition-avgposition;
		derivitave = error - prevError;

		totalerror+=error;


		double motorpower= (error*kP + totalerror * kI + derivitave * kD);
		left_mg.move_velocity(motorpower);
		right_mg.move_velocity(motorpower);


		prevError-error;
		pros::delay(20);
		
	}
	left_mg.move_velocity(0);
		right_mg.move_velocity(0);
	
}

void resetdriveEncoders(){
  left_mg.tare_position();
  right_mg.tare_position();
}


void firstroute(){
    if (master.get_digital_new_press(DIGITAL_DOWN)){
      intake.move(126);
      left_mg.move_relative(7020,126);
      right_mg.move_relative(7020,126);
      left_mg.move_relative(-360,126);
      right_mg.move_relative(360,126);
      /*left_mg.move_relative(10530,126);
      right_mg.move_relative(10530,126);
       left_mg.move_relative(45,126);
        right_mg.move_relative(-45,126);
        left_mg.move_relative(7847.775,126);
        left_mg.move_relative(7847.775,126);
        left_mg.move_relative(-45,126);
        right_mg.move_relative(45,126);
        left_mg.move_relative(7020,126);
      right_mg.move_relative(7020,126);
      left_mg.move_relative(-135,126);
        right_mg.move_relative(135,126);
        left_mg.move_relative(7847.775,126);
        left_mg.move_relative(7847.775,126);
        left_mg.move_relative(-45,126);
        right_mg.move_relative(45,126);
        left_mg.move_relative(2*7847.775,126);
      left_mg.move_relative(2*7847.775,126);
pros::delay(20);
      /*24,turn 45, move 26.83,move 24. turn 135, move 26.83, 53.665*/}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	
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
	  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		
    if (master.get_digital(DIGITAL_L1)){
      IntakeTempSpeed = 1;
      
    }
    else if (master.get_digital(DIGITAL_L2))
    {
      IntakeTempSpeed = 2;
    }
    else if (master.get_digital(DIGITAL_X))
    {
      IntakeTempSpeed = 0;
    }
    else{
      IntakeTempSpeed = 0;
    }
   if (IntakeTempSpeed==1){
      intake.move(126);
      pros::delay(20);
   }
    if (IntakeTempSpeed==2){
      intake.move(-126);
      pros::delay(20);
   }
if (IntakeTempSpeed==0){
      intake.move(0);
      pros::delay(20);
   }
		if (master.get_digital_new_press(DIGITAL_A)) {
			left_mg.set_brake_mode_all(MOTOR_BRAKE_HOLD);
			right_mg.set_brake_mode_all(MOTOR_BRAKE_HOLD);
            master.print(0, 0, "                             ");
            
            double start = pros::millis();
            left_mg.move_relative(7020, 126);
			right_mg.move_relative(7020, 126);
			pros::delay(20);


            master.print(0, 0, "%.0f                             ", (pros::millis() - start));
            pros::delay(500);
            master.print(1, 0, "%.2f                             ", inertial.get_heading());
            pros::delay(500);
		}
if (master.get_digital_new_press(DIGITAL_B)) {
			left_mg.set_brake_mode_all(MOTOR_BRAKE_HOLD);
			right_mg.set_brake_mode_all(MOTOR_BRAKE_HOLD);
            master.print(0, 0, "                             ");
            
            double start = pros::millis();
            left_mg.move_relative(180, 126);
			right_mg.move_relative(-180, 126);
			pros::delay(20);


            master.print(0, 0, "%.0f                             ", (pros::millis() - start));
            pros::delay(500);
            master.print(1, 0, "%.2f                             ", inertial.get_heading());
            pros::delay(500);
		}
     if (master.get_digital_new_press(DIGITAL_DOWN)){
      
     left_mg.set_brake_mode_all(MOTOR_BRAKE_HOLD);
			right_mg.set_brake_mode_all(MOTOR_BRAKE_HOLD);
            master.print(0, 0, "                             ");
            
            double start = pros::millis();
            left_mg.move_relative(10530,126);
      right_mg.move_relative(10530,126);
       left_mg.move_relative(45,126);
        right_mg.move_relative(-45,126);
        left_mg.move_relative(7847.775,126);
        left_mg.move_relative(7847.775,126);
        left_mg.move_relative(-45,126);
        right_mg.move_relative(45,126);
        left_mg.move_relative(7020,126);
      right_mg.move_relative(7020,126);
      left_mg.move_relative(-135,126);
        right_mg.move_relative(135,126);
        left_mg.move_relative(7847.775,126);
        left_mg.move_relative(7847.775,126);
        left_mg.move_relative(-45,126);
        right_mg.move_relative(45,126);
        left_mg.move_relative(2*7847.775,126);
      left_mg.move_relative(2*7847.775,126);
pros::delay(20);
			pros::delay(20);


            master.print(0, 0, "%.0f                             ", (pros::millis() - start));
            pros::delay(500);
            master.print(1, 0, "%.2f                             ", inertial.get_heading());
            pros::delay(500);

     }
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);                     // Sets right motor voltage
		pros::delay(20);                             // Run for 20 ms then update
	}



}