/*******INFO*******
Example #3: Segway-Encoders.c Rotation sensor program for a LEGO NXT Segway
Author: Laurens Valk (robotsquare.com)
License: Use the examples as you like, but give credit to the original author. See driver file for more details.
Full instructions: http://robotsquare.com/2012/02/13/tutorial-segway-with-robotc/
***************/

/*CHOOSE A SENSOR: Comment out the sensors that you do not have*/
//#define HiTechnic_Gyro
#define DexterIndustries_dIMU
//#define MicroInfinity_Cruizcore
//#define MindSensors_IMU

/*SELECT SENSOR PORT (S1, S2, S3 or S4), and WHEEL DIAMETER (milimeters).*/
const tSensors Gyro = S2;
const float your_wheel_diameter = 42;

/*These are driver files to read gyro sensors. Make sure the path is correct.
  You may delete these two lines if you use the HiTechnic Gyro*/
#include "drivers/dexterind-imu.h"
#include "drivers/microinfinity-cruizcore.h"
#include "drivers/mindsensors-imu.h"
/*This is the Segway Driver code. Place in same directory as this program*/
#include "segway-driver-lv.h"

task main()
{
  //Start balancing and wait for configuration to finish
  StartTask(balancing);
  while(starting_balancing_task){}

  while(true)
  {
	  speed = 60;
	  while(nMotorEncoder[motorB] > -1000){wait1Msec(100);}
	  //#1: Polling 10 times/sec is good enough -- Save processor power for the balancing task.
	  //#2: Going forwards means that the motors run backwards, because of their orientation.
	  //    So to go forward for 1000 degrees, the encoder must reach -1000 degrees.
	  speed = -60;
	  while(nMotorEncoder[motorB] < 0){wait1Msec(100);}
  }

}
