/*******INFO*******
Example #1: Segway-Wall-Avoidance.c Wall avoidance program for a LEGO NXT Segway (Includes description of commands to control Segway movement)
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
  StartTask(balancing);
  while(starting_balancing_task){}
  //It's that simple - now go program something cool below!

  /*Here's how to control the Segway.

  MOVING AROUND
  - Adjust <steering> to steer.
  	   steering = 0  ; //Go Straight
  	   steering = 10 ; //Turn Right
       steering = 17;  //Turn Right quicklier
  	   steering = -10; //Turn Left
       etc...

   - Adjust <speed> for moving forward and backwards.
       Its value is the motor power that the robot will attempt to maintain

  	   speed = 0;     //Stand still
  	   speed = -20 ; //Go backward at 20% of motor power
       speed = 50;   //Go forward at half of maximum motor power (recommended)
       etc...

    - Adjust <acceleration> to change how quickly your robot will reach a certain speed.

       acceleration = 60; means that if your current speed is -20 and you change it
       to 40, it will take 1 seconds to get to that speed. You don't need to change this
       in your program, just set it once. Setting <acceleration = 60;> is recommended.

  TIPS & TRICKS
  - Don't mess with motor B and C!
  - You can read the motor encoders like you would normally do. Just don't reset them.
  - 'speed' sets the segway motor speed - This means you go faster if you use bigger wheels.
  - If you use those big RCX/Motorcycle wheels, say, greater than 60 mm in diameter, consider
    building a slightly bigger Segway, with better supported wheels and a greater wheelspan.
    That improves balancing a great deal!
   */

   //////////////////////////////////////////////
   //
   // EXAMPLE #1: Wall avoidance



   const tSensors Sonar = S4;
   SensorType[Sonar] = sensorSONAR;

   acceleration = 60;
   steering = 0;
   wait1Msec(3000);//We wait three seconds just to be sure we're up and running
   PlaySound(soundBeepBeep);

   speed = 50;

   //Every 100 ms, we check if the sensor has spotted anything closer than 60 cm.
   //When so, the robot goes backwards, turns, and continues its way. Note how the
   //speed and steering variables are used to manipulate the robot's movement.

   while(true)
   {
     if(SensorValue[Sonar] < 60)
     {
       speed = -50;
       wait1Msec(2000);
       speed = 0;
       wait1Msec(500);
       steering = 10;
       wait1Msec(1000);
       steering = 0;
       speed = 50;
     }
     wait1Msec(100);
   }
}
