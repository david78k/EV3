/*******INFO*******
Example #2: Segway-NoAction.c Sample Program for a LEGO NXT Segway
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
  You may delete these three lines if you use the HiTechnic Gyro*/
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

  /*
  Place your own code here.

  Modify <speed> and <steering> to make the Segway move.
  For example:

    speed = 30;
    steering = -7;

  The default is 0 for both, which means it stands still.

  For further instructions:
  http://robotsquare.com/2012/02/13/tutorial-segway-with-robotc/
  */
}
