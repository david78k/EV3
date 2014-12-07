/*

LegWay

self-balancing robot

by Steve Hassenplug
http://www.teamhassenplug.org

*/

int MotorSpeedArray[32] =  {1,1,1,1,1,1,1,1, // motor speed 0-7 = forward, 0 = fast, 7 = slow
							3,3,3,3,3,3,3,3, // motor speed 8 = stop
							2,2,2,2,2,2,2,2, // motor speed 9-16 = reverse, 9 = slow, 16 = full
							0,0,0,0,0,0,0,0}; // 24 = float

#include <conio.h>
#include <unistd.h>
#include <dsensor.h>
#include <dmotor.h>
#include <rom/system.h>
#include <dsound.h>
#include <dbutton.h>

int main(int argc, char *argv[])
{
	int L1;
	int MotorA = 0;

	int MotorSetting;

	int L3;
	int MotorC = 0;

	int MotorRunningValue;

	long CheckTime;		// adjust center point
	long AdjustTime;	// adjust for falling
	long FallTime;		// last time < full speed

	int CenterPoint1;
	int CenterPoint3;

// walker variables
	int MoveForward = 1;
	int ForwardDrop = 16;

// spinner variables
	long NextSpin;
	int TurnDirection = 1;
	int EnableSpinner = 0;

	ds_active(&SENSOR_1);
	ds_active(&SENSOR_3);

	motor_a_dir(3);
	motor_c_dir(3);
	motor_a_speed(255);
	motor_c_speed(255);

	L1 = 0;
	L3 = 0;
	MotorSetting = 8;

	msleep(500);  // wait for sensors to power up

	CenterPoint1 = LIGHT_1; // pick a starting center point
	CenterPoint3 = LIGHT_3;

	if ((CenterPoint1 < 3) || (CenterPoint3 < 3)) // only 1 sensor connected
	{
		MoveForward = 0; // do not move forward
		ForwardDrop = 100;
	}

	dsound_system(DSOUND_BEEP);

	while(PRESSED(dbutton(),BUTTON_RUN))
	{
		EnableSpinner = 1;
		MoveForward = 0;
	}


	FallTime = get_system_up_time();
	CheckTime = 0;
	AdjustTime = 0;
	NextSpin = get_system_up_time() + 5000;


	while(FallTime + 1000 > get_system_up_time())
	{

// spinner code
		if ((get_system_up_time()>NextSpin) && (EnableSpinner == 1))
		{
			MoveForward = MoveForward + TurnDirection;
			NextSpin = get_system_up_time() + 1000;
			if ((MoveForward < -5) || ( MoveForward > 5))
			{
				TurnDirection = - TurnDirection;
				NextSpin = get_system_up_time() + 5000;
			}
		}
// end spinner

		if (get_system_up_time()>AdjustTime) // re-calculate center point based on motorsettings
		{
			CenterPoint1 = (CenterPoint1 + MotorSetting - 8);
			CenterPoint3 = (CenterPoint3 + MotorSetting - 8);
			AdjustTime = get_system_up_time() + 50;
		}

		if (get_system_up_time()>CheckTime)
		{

			L1 = LIGHT_1;
			L3 = LIGHT_3;

			if (L1 > L3)
			{
				MotorSetting = (CenterPoint1 - L1) / 2 + 8;
				// divide by 2 to slow response down
			}
			else
			{
				MotorSetting = (CenterPoint3 - L3) / 2 + 8;
			}

			// check boundry
			if (MotorSetting < 0) MotorSetting = 0;
			if (MotorSetting > 16) MotorSetting = 16;

			// L1 over black
			if (L1 < L3 - ForwardDrop) MotorA = -MotorSetting; // stop motor A
			else
			{
				if (EnableSpinner == 1)
					MotorA = - MoveForward; // move opposite direction for spin
				else
					MotorA = MoveForward; // move forward
			}

			// L3 over black
			if (L3 < L1 - ForwardDrop) MotorC = -MotorSetting; // stop motor c
			else MotorC = MoveForward; // move forward

			CheckTime = get_system_up_time() + 1;
		}

		// less than full power
		if (MotorSetting > 0 && MotorSetting < 16) FallTime = get_system_up_time();

		MotorRunningValue = (get_system_up_time() & 7);
		motor_a_dir(MotorSpeedArray[MotorSetting + MotorRunningValue - MotorA]);
		motor_c_dir(MotorSpeedArray[MotorSetting + MotorRunningValue - MotorC]);

		if (((get_system_up_time() >> 8) & 2) == 0) // show LegWay on desplay
			cputs("LEG");
		else
			cputs(" WAY");
	}
	motor_a_dir(3);  // stop after fall time expires
	motor_c_dir(3);
	ds_passive(&SENSOR_1);
	ds_passive(&SENSOR_3);
	return 0;
}

