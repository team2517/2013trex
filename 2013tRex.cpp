#include "WPIlib.h"
#include "Math.h"
#define PI 3.14159265

//Remote system processor 5200?

float normalizePower(float);

class DefaultRobot : public SimpleRobot {
	Joystick joystick;
	Joystick armControl;
	CANJaguar jagA;
	CANJaguar jagB;
	CANJaguar jagC;
	CANJaguar jagD;
	Solenoid tiltA; //Pneumatic at the base of the arm that controls tilt.	
	Solenoid tiltB;
	Solenoid liftA; //The pneuamtic at the first joint that controls lift.	
	Solenoid liftB;
	Solenoid clampA; //The pneumatic at the end of the arm that controls the clamp.	
	Solenoid clampB;
	Compressor compressor;
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
				 * 
				 * 
				 * 45 = d = rear right 
				 * 30 = c = rear left
				 * 5 = a = front left
				 * 2 = b = front right
				 */

				joystick(1),
				armControl(2),
				jagA(5), 
				jagB(2), 
				jagC(30), 
				jagD(45), 
				compressor(1, 3), 
				tiltA(1), 
				tiltB(2), 
				liftA(3), 
				liftB(4),
				clampA(5), 
				clampB(6)

	{
		Watchdog().SetExpiration(1);
		compressor.Start();
	}
	void Autonomous(void) 
	{

	}

	void OperatorControl(void) 
	{
		Watchdog().SetEnabled(true);
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		

		Watchdog().Feed();

		while (IsOperatorControl()) 
		{
			if (-.1 < joystick.GetRawAxis(3) && (joystick.GetRawAxis(3) < .1)) 
			{
				phi = 0;
			} 
			else 
			{
				phi = joystick.GetRawAxis(3);
			}

			leftJoyX = joystick.GetRawAxis(1);
			leftJoyY = -joystick.GetRawAxis(2);
			radius = sqrt(pow(leftJoyX, 2) + pow(leftJoyY, 2));
			
			//Left joystick strafe tolerance
			if ((-.1 < leftJoyX) && (leftJoyX < .1)) 
			{
				leftJoyX = 0;
			}
			if ((-.1 < leftJoyY) && (leftJoyY < .1)) 
			{
				leftJoyY = 0;
			}

			//theta NaN handling for X/Y axis movement and calculation
			//Left stick - X axis
			if ((leftJoyY == 0) && (leftJoyX != 0)) 
			{
				if (leftJoyX > 0) 
				{
					theta = 0;
				} 
				else if (leftJoyX < 0) 
				{
					theta = PI;
				}
				//Left stick - Y axis
			} 
			else if ((leftJoyX == 0) && (leftJoyY != 0)) 
			{
				if (leftJoyY > 0) 
				{
					theta = PI / 2;
				} 
				else if (leftJoyY < 0) 
				{
					theta = (3 * PI) / 2;
				}
				//No movement
			} 
			else if ((leftJoyY == 0) && (leftJoyX == 0)) 
			{
				theta = 0;
			} 
			else 
			{
				theta = atan((leftJoyY) / (leftJoyX));
			}

			//if in Quadrant 2 or 3 add 180 degrees, if in Quadrant 4 add 360
			if (((leftJoyX < 0) && (leftJoyY > 0)) || ((leftJoyX < 0) && (leftJoyY < 0))) 
			{
				theta += PI;
			} else if ((leftJoyX > 0) && (leftJoyY < 0)) 
			{
				theta += (2 * PI);
			}

			

			//Power equations
			outA = ((radius) * (sin(theta + (PI / 4))) + phi);
			outB = ((radius) * (cos(theta + (PI / 4))) + phi);
			outC = -((radius) * (cos(theta + (PI / 4))) - phi);
			outD = -((radius) * (sin(theta + (PI / 4))) - phi);

			//Output to motors
			jagA.Set(normalizePower(outA));
			jagB.Set(normalizePower(outB));
			jagC.Set(normalizePower(outC));
			jagD.Set(normalizePower(outD));

			//Wait(.05);

			
			if (armControl.GetRawButton(3)) //Lift Up/Extended			
			{
				liftA.Set(true); //Lift extends when button 3 is pressed and tilt is retracted.				
				liftB.Set(false); 				
			}
			if (armControl.GetRawButton(2)) //Lift Down/Retracted			
			{
				liftA.Set(false);
				liftB.Set(true);
			}
			
			
			if (armControl.GetRawButton(9)) //Tilt Forward/Extended			
			{
				tiltA.Set(true); //Tilt retracts when button 9 is pressed and lift is retracted.				
				tiltB.Set(false); 				
			} 
			else if (armControl.GetRawButton(8)) //Tilt Backward/Retracted			
			{
				tiltA.Set(false); 				
				tiltB.Set(true);
			}
			
			if (armControl.GetRawButton(1)) //Deploy Minibot/Extended		
			{
				clampA.Set(true); //Clamp opens when button 1 is pressed.			
				clampB.Set(false);
			} 
			else 
			{
				clampA.Set(false);
				clampB.Set(true);
			}

			

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
			dsLCD->UpdateLCD();
			Watchdog().Feed();
		}
	}
};

float normalizePower(float power)
{
	if(power > 1)
	{
		power = 1;
	}
	else if(power < -1)
	{
		power = -1;
	}
	
	return power;
}

START_ROBOT_CLASS(DefaultRobot)
;

