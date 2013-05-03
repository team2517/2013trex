#include "WPIlib.h"
#include "Math.h"
#define PI 3.14159265


class DefaultRobot: public SimpleRobot {
	Joystick joystick;
	Jaguar jagA;
	Jaguar jagB;
	Jaguar jagC;
	Jaguar jagD;
	double theta;
	double radius;
	double phi;
	double leftJoyX;
	double leftJoyY;
	double outA;
	double outB;
	double outC;
	double outD;
public:
	DefaultRobot(void) :
		/*
		 * A = 2
		 * B = 3
		 * C = 1
		 * D = 4
		 */
		
		joystick(1), 
		jagA(2), //invert
		jagB(3), //invert
		jagC(1), 
		jagD(4)	
	
	{
		Watchdog().SetExpiration(1);	
	}
	//loader piston on flipper solenoid
	void Autonomous(void) {
	}

	void OperatorControl(void) {
		Watchdog().SetEnabled(true);
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		while (IsOperatorControl()) {
			if(-.1 < joystick.GetRawAxis(3) && (joystick.GetRawAxis(3) < .1)){
				phi = 0;
			}else{
				phi = joystick.GetRawAxis(3);
			}
			
			leftJoyX = joystick.GetRawAxis(1);
			leftJoyY = -joystick.GetRawAxis(2);
			radius = sqrt(pow(leftJoyX, 2) + pow(leftJoyY, 2));
			
			//Left joystick strafe tolerance
			if ((-.1 < leftJoyX) && (leftJoyX < .1)) {
				leftJoyX = 0;
			}
			if ((-.1 < leftJoyY) && (leftJoyY < .1)) {
				leftJoyY = 0;
			}

			//theta NaN handling for X/Y axis movement and calculation
			//Left stick - X axis
			if ((leftJoyY == 0) && (leftJoyX != 0)) {
				if (leftJoyX > 0) {
					theta = 0;
				} else if (leftJoyX < 0) {
					theta = PI;
				}
			//Left stick - Y axis
			} else if ((leftJoyX == 0) && (leftJoyY != 0)) {
				if (leftJoyY > 0) {
					theta = PI / 2;
				} else if (leftJoyY < 0) {
					theta = (3 * PI) / 2;
				}
			//No movement
			} else if ((leftJoyY == 0) && (leftJoyX == 0)) {
				theta = 0;
			} else {
				theta = atan((leftJoyY) / (leftJoyX));
			}

			//if in Quadrant 2 or 3 add 180 degrees, if in Quadrant 4 add 360
			if (((leftJoyX < 0) && (leftJoyY > 0)) || ((leftJoyX < 0)
					&& (leftJoyY < 0))) {
				theta += PI;
			} else if ((leftJoyX > 0) && (leftJoyY < 0)) {
				theta += (2 * PI);
			}
				
			//Power equations
			outA = ((radius) * (sin(theta + (PI / 4))) + phi);
			outB = ((radius) * (cos(theta + (PI / 4))) + phi);
			outC = ((radius) * (cos(theta + (PI / 4))) - phi);
			outD = ((radius) * (sin(theta + (PI / 4))) - phi);

			//Output to motors
			if (outA > 1) {
				jagA.Set(-1);
			} else if (outA < -1) {
				jagA.Set(1);
			} else {
				jagA.Set(-outA);
			}
			if (outB > 1) {
				jagB.Set(-1);
			} else if (outB < -1) {
				jagB.Set(1);
			} else {
				jagB.Set(-outB);
			}
			if (outC > 1) {
				jagC.Set(1);
			} else if (outC < -1) {
				jagC.Set(-1);
			} else {
				jagC.Set(outC);
			}
			if (outD > 1) {
				jagD.Set(1);
			} else if (outD < -1) {
				jagD.Set(-1);
			} else {
				jagD.Set(outD);
			}
		}
	}
};

START_ROBOT_CLASS(DefaultRobot);
