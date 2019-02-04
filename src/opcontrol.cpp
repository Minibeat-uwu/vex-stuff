#include "main.h"
using namespace std;
using namespace pros;
#define PORT_TOP_LEFT 'E'
#define PORT_BOTTOM_LEFT 'F'
#define PORT_TOP_RIGHT 'C'
#define PORT_BOTTOM_RIGHT 'D'
#define pi 3.14
#define KP 0.8
#define KI 0.001f
#define KD 0.1f
#define threshold 1000
#define turn_threshold 20
#define degreesPerRotation 360
#define wheelDiameter 4 //4.0" wheels
double circ = pi*wheelDiameter;
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

 pros::Controller master(pros::E_CONTROLLER_MASTER);
 pros::Motor left_mtr1(1);
 pros::Motor left_mtr2(2);
 pros::Motor right_mtr1(3);
 pros::Motor right_mtr2(4);
 pros::ADIEncoder left_sensor (PORT_TOP_LEFT, PORT_BOTTOM_LEFT, false);
 pros::ADIEncoder right_sensor (PORT_TOP_RIGHT, PORT_BOTTOM_RIGHT, false);


 //Inputs inches, outputs proper tick value needed
 double inchesToDegrees(double inches){
	return (inches/circ)*degreesPerRotation;
}

double degreesToInches(double degrees){
	return (degrees/degreesPerRotation)*circ;
}

// Used to relate position value (meters) to speed/velocity (meters per second) since they are two different units.
double map(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Found 90 degree turn to be 0.6875 rotations.
// 1 for left turn, 2 for right turn


	//The actual move straight function that runs the motors
void moveStraight(double distance_in_inches){
		//Resets the encoder value
		//reads encoder values
		double left_pos = left_sensor.get_value();
		double right_pos = right_sensor.get_value();
		double setpoint, error, reference,speed;
		setpoint = inchesToDegrees(distance_in_inches);
		speed = 67; //max speed

	//	while(abs(left_pos1)<setpoint && abs(right_pos1) < setpoint){
	/* Setpoint, Error, Reference values are position values (in degrees for encoder units).
		 Proportional Controller gain is used for modifying speed/velocity depending on how much error remains (larger error = lower speed required, vice-versa)
	*/
		while(true){
			error = setpoint - left_pos; // = setpoint - right_pos1 (in degrees)
			cout << "error: " << error << "			left position: " << left_pos << "			right position: " << right_pos << "\n";
			delay(10);
			if (abs(error)<=threshold)
				break; //don't think this condition will work because the motor spins pretty fast, so it will probably overshoot and undershoot the target position forever.

			// not really sure how to use the KP constant on the error, since we are changing speed which is in different units than the error's units (meters/s vs meters).
			//Play around with different ways of using error and KP constant to change speed
		//	speed = map(error * KP, -setpoint, setpoint, -127,127);


			left_mtr1= speed; //move() is same as move_velocity() command
			left_mtr2= speed; //move() is same as move_velocity() command
			right_mtr1 = speed;
			right_mtr2 = speed;
			left_pos = left_sensor.get_value(); //Constantly reads the encoder value
			right_pos = right_sensor.get_value();
			delay(10);
		}
		left_mtr1 = 0;
		left_mtr2 = 0;
		right_mtr1 = 0;
		right_mtr2 = 0;
		delay(10);
	}

	//Turn function
	void resetEncoder(){
		left_mtr1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		left_mtr2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		right_mtr1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		right_mtr2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	  left_sensor.reset(); // The encoder is now zero again
		right_sensor.reset(); // The encoder is now zero again
	}

  void turn90degrees(int direction){
    double setpoint = 0.6875 * degreesPerRotation; //How many rotation it needs to turn 90 degrees
  	//Resets the encoder value
  	//reads encoder values
  	double left_pos = left_sensor.get_value();
  	double right_pos = right_sensor.get_value();

  	double error, reference,speed;
  	speed = 30; //max speed

  	if (direction == 1){
  			left_mtr1.set_reversed(false);
  			right_mtr1.set_reversed(true);
  		}
  	else if (direction ==2)
  		{
  			right_mtr1.set_reversed(false);
  			left_mtr1.set_reversed(true);
  		}
  	while(true){
  			if (direction ==1)
  				error = setpoint - left_pos; // = setpoint - right_pos1 (in degrees)
  			else if (direction==2)
  				error = setpoint - right_pos;
  			cout << "error: " << error << "			left position: " << left_pos << "			right position: " << right_pos << "\n";
  			delay(10);
  			if (abs(error)<=turn_threshold)
  				//break; //don't think this condition will work because the motor spins pretty fast, so it will probably overshoot and undershoot the target position forever.

  			// not really sure how to use the KP constant on the error, since we are changing speed which is in different units than the error's units (meters/s vs meters).
  			//Play around with different ways of using error and KP constant to change speed
        //kp=proportional control: Basically trial and error and test the distance.
        //
        speed = map(error * KP, -setpoint, setpoint, -127,127);

  			left_mtr1= speed; //move() is same as move_velocity() command
  			left_mtr2= speed; //move() is same as move_velocity() command
  			right_mtr1 = speed;
  			right_mtr2 = speed;
  			left_pos = left_sensor.get_value(); //Constantly reads the encoder value
  			right_pos = right_sensor.get_value();
  			delay(50);
  			if(left_pos>=setpoint||left_pos<=-setpoint||right_pos>=setpoint||right_pos<=-setpoint){
          resetEncoder();
          break;
  			}
  		}
  		left_mtr1 = 0;
  		left_mtr2 = 0;
  		right_mtr1 = 0;
  		right_mtr2 = 0;
  		delay(50);
  	}

void opcontrol() {

	resetEncoder();
	//Basic movement system vv
	/*
		pros::Controller master(pros::E_CONTROLLER_MASTER);
		pros::Motor left_mtr1(left_mtr1_port,E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
		pros::Motor left_mtr2(2);
		pros::Motor right_mtr1(right_mtr1_port,E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
		pros::Motor right_mtr2(4);

		while (true) {
			int left = master.get_analog(ANALOG_LEFT_Y);
			int right = master.get_analog(ANALOG_RIGHT_Y);

			if(abs(left)<5){
				left_mtr1.move_velocity(0);
				left_mtr2.move_velocity(0);
			}else{
				left_mtr1.move_velocity(left);
				left_mtr2.move_velocity(left);
				int left_pos = left_mtr1.get_position();
				cout << "Left Position: " << left_pos << endl;
				delay(100);
			}

			if(abs(right)<5){
				right_mtr1.move_velocity(0);
				right_mtr2.move_velocity(0);
			}else{
				right_mtr1.move_velocity(right);
				right_mtr2.move_velocity(right);
			  int	right_pos = right_mtr1.get_position();
				cout << "Right Position (rotations): " << right_pos << endl;
				delay(100);

			}
			pros::delay(20);
		}
		*/

	//moveS`traight(100);
	//delay(100);
	turn90degrees(1);
	delay(200);
	turn90degrees(2);
	delay(200);
}


	//int target =
//	while (true){}



/*

//The actual move straight function that runs the motors
void moveStraight(int inches, int speed, int direction){
	//Resets the encoder value
	SensorValue[driveE] = 0;
	int currentValue = 0;
	int targetTicks = inchesToTicks(inches);
	inchesToTicks(inches);
	while(abs(currentValue)<targetTicks){
		motor[Ldrive] = speed*direction;
		motor[Rdrive] = speed*direction;
		currentValue = SensorValue[driveE]; //Constantly reads the encoder value
	}
	motor[Ldrive] = 0; //Stops drive motors after target it reached
	motor[Rdrive] = 0;
}

//Turn function that either turns 90 degrees cw or ccw depending on direction input
void turn(int direction){
	SensorValue[driveE] = 0;
	int currentValue = 0;
	int targetTicks = 240;
	while(abs(currentValue)<targetTicks){
		motor[Ldrive] = -40*direction;
		motor[Rdrive] = 40*direction;
		currentValue = SensorValue[driveE];
	}
	motor[Ldrive] = 0;
	motor[Rdrive] = 0;
}
*/
