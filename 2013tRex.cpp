#include "WPIlib.h"
#include "Math.h"
#include <time.h>
#define PI 3.14159265


class DefaultRobot: public SimpleRobot {
	Joystick joystick;
	Joystick joystick2;
	CANJaguar jagA;
	CANJaguar jagB;
	CANJaguar jagC;
	CANJaguar jagD;
	CANJaguar jagWindowMotor;
	CANJaguar shootFront;
	CANJaguar shootRear;
	Solenoid suctionA;
	Solenoid suctionB;
	Solenoid loaderA;
	Solenoid loaderB;
	Solenoid hopperGateA;
	Solenoid hopperGateB;
	Solenoid lifterA;
	Solenoid lifterB;
	AnalogChannel encoder;
	AnalogChannel pValue;
	AnalogChannel iValue;
	AnalogChannel dValue;
	Compressor compressor;
	Timer timer;
	int lifterStep;
	double theta;
	double radius;
	double phi;
	double leftJoyX;
	double leftJoyY;
	double outA;
	double outB;
	double outC;
	double outD;
	double shooterSpeed;
	double armPosition;
	double targetPosition;
	double armPositionTrans;
	double targetSpeed;
	double motorSpeed;
	double maxPower;
	bool button3Pressed;
	bool button2Pressed;
	bool lifterStart;
public:
	DefaultRobot(void) :
		/*
		 * 3 = green   = d = rear right 
		 * 6 = red     = c = rear left
		 * 11 = white  = a = front left
		 * 10 = yellow = b = front right
		 * 
		 * hopper gate solenoid have placeholder values
		 */
		
		joystick(1), 
		joystick2(2),
		jagA(11), //invert
		jagB(10), //invert
		jagC(6), 
		jagD(3),
		jagWindowMotor(4),
		shootFront(7),
		shootRear(2),
		suctionA(1),
		suctionB(2),
		loaderA(3),
		loaderB(4),
		hopperGateA(5),
		hopperGateB(6),
		lifterA(7),
		lifterB(8),
		encoder(1),
		pValue(2),
		iValue(3),
		dValue(4),
		compressor(1, 1)
		
	
	{
		Watchdog().SetExpiration(1);
		compressor.Start();	
	}
	//loader piston on flipper solenoid
	void Autonomous(void) {
		
	}

	void OperatorControl(void) {
		Watchdog().SetEnabled(true);
		timer.Start();
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		double ptemp=3.0;
		double itemp=0.0;
		double dtemp=0.0;
		jagA.ChangeControlMode(jagA.kSpeed);
		jagA.ConfigEncoderCodesPerRev(250);
		jagA.SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
		jagA.SetPID(1.37,0.0,5);
		jagA.EnableControl();
		Watchdog().Feed();
		jagB.ChangeControlMode(jagB.kSpeed);
		jagB.ConfigEncoderCodesPerRev(250);
		jagB.SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
		jagB.SetPID(1.37,0.0,5);
		jagB.EnableControl();
		Watchdog().Feed();
		jagC.ChangeControlMode(jagC.kSpeed);
		jagC.ConfigEncoderCodesPerRev(250);
		jagC.SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
		jagC.SetPID(1.37,0.0,5);
		jagC.EnableControl();
		Watchdog().Feed();
		jagD.ChangeControlMode(jagD.kSpeed);
		jagD.ConfigEncoderCodesPerRev(250);
		jagD.SetSpeedReference(CANJaguar::kSpeedRef_Encoder	);
		jagD.SetPID(1.37,0.0,5);
		jagD.EnableControl();
		
		Watchdog().Feed();
		shooterSpeed = 0.0;
		maxPower = 275;
		lifterStep = 0;
		button3Pressed = false;
		button2Pressed = false;
		lifterStart = false;
		
		while (IsOperatorControl()) {
			if(-.1 < joystick.GetRawAxis(3) && (joystick.GetRawAxis(3) < .1)){
				phi = 0;
			}else{
				phi = joystick.GetRawAxis(3);
			}
			
			leftJoyX = joystick.GetRawAxis(1);
			leftJoyY = -joystick.GetRawAxis(2);
			radius = sqrt(pow(leftJoyX, 2) + pow(leftJoyY, 2));
			
			//i must be 0 or negative!!!
			if(joystick2.GetRawButton(5)){
				ptemp = pValue.GetAverageVoltage();
				itemp = iValue.GetAverageVoltage();
				dtemp = dValue.GetAverageVoltage();
				jagA.DisableControl();
				jagA.SetPID(ptemp, itemp, dtemp);
				jagA.EnableControl();
				jagB.DisableControl();
				jagB.SetPID(ptemp, itemp, dtemp);
				jagB.EnableControl();
				jagC.DisableControl();
				jagC.SetPID(ptemp, itemp, dtemp);
				jagC.EnableControl();
				jagD.DisableControl();
				jagD.SetPID(ptemp, itemp, dtemp);
				jagD.EnableControl();
			}
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
			
			//Test drive buttons
			/*
			if (joystick.GetRawButton(3)) {
				theta = 0;
				radius = .5;
			} else if (joystick.GetRawButton(4)) {
				theta = PI / 2;
				radius = .35;
			} else if (joystick.GetRawButton(1)) {
				theta = PI;
				radius = .5;
			} else if (joystick.GetRawButton(2)) {
				theta = (3 * PI) / 2;
				radius = .35;
			}
			*/
			
			
			//Power equations
			outA = ((radius) * (sin(theta + (PI / 4))) + phi) * maxPower;
			outB = ((radius) * (cos(theta + (PI / 4))) + phi) * maxPower;
			outC = ((radius) * (cos(theta + (PI / 4))) - phi) * maxPower;
			outD = ((radius) * (sin(theta + (PI / 4))) - phi) * maxPower;

			//Output to motors
			if (outA > maxPower) {
				jagA.Set(-maxPower);
			} else if (outA < -maxPower) {
				jagA.Set(maxPower);
			} else {
				jagA.Set(-outA);
			}
			if (outB > maxPower) {
				jagB.Set(-maxPower);
			} else if (outB < -maxPower) {
				jagB.Set(maxPower);
			} else {
				jagB.Set(-outB);
			}
			if (outC > maxPower) {
				jagC.Set(maxPower);
			} else if (outC < -maxPower) {
				jagC.Set(-maxPower);
			} else {
				jagC.Set(outC);
			}
			if (outD > maxPower) {
				jagD.Set(maxPower);
			} else if (outD < -maxPower) {
				jagD.Set(-maxPower);
			} else {
				jagD.Set(outD);
			}
			
			//Wait(.05);
			
			//Pneumatics
			
			//Diagnostics output
			//printf("x: %f y: %f phi: %f\n", leftJoyX, leftJoyY, phi);
			//printf("a: %f b: %f c: %f d: %f\n", jagA.Get(), jagB.Get(), jagC.Get(), jagD.Get());
			//printf("theta: %f radius: %f\n", theta, radius);
			
			//normal debug output (in competition) shooter and lifter info feedback
			
			
			//drive debug info, uncomment as needed.
			//dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Motor A speed: %f", jagA.GetSpeed());
			//dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Motor B speed: %f", jagB.GetSpeed());
			//dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Motor C speed: %f", jagC.GetSpeed());
			//dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Motor D speed: %f", jagD.GetSpeed());
			//dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "P Value: %f", pValue.GetAverageVoltage());
			//dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "I Value: %f", iValue.GetAverageVoltage());
			//dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "D Value: %f", dValue.GetAverageVoltage());
			//dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "pTemp: %f", ptemp);
			//dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "iTemp: %f", itemp);
			//dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "dTemp: %f", dtemp);
			dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Throttle: %f", shooterSpeed);
			dsLCD->UpdateLCD();
			Watchdog().Feed();
		}
	}
};

START_ROBOT_CLASS(DefaultRobot);
