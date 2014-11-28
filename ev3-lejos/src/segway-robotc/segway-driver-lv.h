/* LEGO MINDSTORMS NXT Segway program driver

Author: Laurens Valk (robotsquare.com)

License: Use and modify the driver and examples as you like, but give credit to the original author.
         You cannot redistribute this code. You cannot use this code commercially or sell it.
         Just link to the original: http://robotsquare.com/2012/02/13/tutorial-segway-with-robotc/
         If in doubt, try emailing me for permission: laurensvalk _at_ gmail _dot_com

         When you make custom programs, you will likely make modifications to one of the example
         files. When you do, share only the modified example (.c file). Do not distribute this driver(.h)
         with it. Just link to this latest version.

Comments: Send them to laurensvalk _at_ gmail _dot_com

Requirements, details and instructions -- Read the Tutorial:
http://robotsquare.com/2012/02/13/tutorial-segway-with-robotc/

Version history:

- v1.10 10 November 2012  - Updated to be compatible with RobotC 3.54 and 3Rd Party Suite v3.1
													- Added support for Minsensors Absolute IMU. IMU firmware should be at least 1.51.
													- Refer to the Mindsensors IMU user guide for instructions to update the firmware.
													- Fixed 'shake' at startup.
- v1.01 29 March    2012  - 'dt' and 'gear_down_ratio' are now global variables
- v1.00 28 March    2012  - Added support for MicroInfinity gyroscope
                          - Conditional compilation: Only code for your sensor gets compiles. This requires
                            different set up in the sample programs. For more, just see the tutorial.
- v0.98 20 March    2012: - You can now use the code with a Segway with gearing down between wheels
                          - Both supported sensors now use same PID constants
                          - Added section at the beginning of the code for user adjustable settings
- v0.97 07 March    2012: - Loop cycle now uses timer to determine cycle length.
                          - Motor encoder speed is now really defined as degrees per second.
                          - Both changes further increase flexibility of the driver, but
                            the consequence is that the constants have been adjusted to
                            compensate for the changes.
                            NOTE: This does not affect your other programs. You can safely
                            update to this driver version.
- v0.95 02 February 2012: - Minor changes to driver increase time invariancy.
                          - The following variables are now globally accessible to allow parameters
                            to be changed dynamically:
                            gn_dth_dt,gn_th,gn_y,gn_dy_dt,kp,ki,kd,mean_reading
- v0.9  14 November 2011: - Public release
- v0.7  09 November 2011: - Added support for HiTechnic Gyroscope Calibration (Release candidate)
- v0.6  06 November 2011: - Added support for HiTechnic and Dexter Industries Gyroscopes
- v0.5  ~~ November 2011: - Rewriting in driver form using a parallel task
- v0.4  ~~ October  2011: - Rewriting of old coding for readability and efficient processing
- v0.1  ~~ 2010:          - Initial proof of concept Segway Design

Compatible Gyroscopes:
- HiTechnic Gyro NGY1044
- Dexter Industries 6DOF IMU dIMU
- Micro Infinity Cruizcore XG1300L
- Mindsensors AbsoluteIMU-ACG

Segway hardware:
- Segway building instructions compatible with this program:
  http://robotsquare.com/2012/02/12/tutorial-building-segway/

Files:
The task in this driver should run parallel to the main program task.
It will handle all balancing control and leaves the main task free for
custom user programs, allowing the Segway to move around and, for example
avoiding walls. The following examples make use of this driver:

 - Example #1: Segway-Explanation(Wall-Avoidance).c (Includes description of commands to control Segway movement)
 - Example #2: Segway-NoAction.c
 - Example #3: Segway-Encoders.c

This Segway driver uses Sensor drivers (v2.4 or later) from the RobotC driver suite:
http://botbench.com/blog/robotc-driver-suite/
You need the sensor driver suite at all times, even if you use the HiTechnic Gyro!
*/

int steering = 0;
int acceleration = 50;
int speed = 0;
bool starting_balancing_task = true;

//GLOBAL VARIABLE SETUP
float gn_dth_dt,gn_th,gn_y,gn_dy_dt,kp,ki,kd,mean_reading,gear_down_ratio,dt;

#ifdef HiTechnic_Gyro
//Prototype (see full code below)
int calibrate_hitechnic();
#endif

task balancing()
{
  ///////////////////////////
  //ADVANCED USER CONTROL
  ///////////////////////////

  /*
  The following values can be modified for advanced control. Many users will not use these features,
  which is why I do not require these to be set in the main program. You can just modify them here.
  if you are unsure about what they do, just leave them at the default value.
  */

  // Set gearing down ratio between motor and wheels (e.g. 5x slow down: 40z / 8z = 5)
  // The default is 1, no gearing down.
  gear_down_ratio = 1;

  // Set the time each loop cycle should last. You can set it up to 0.03 seconds or even higher, if you really
  // need to. If you add code to the control loop below (such as to read another sensor), make sure that
  // all code can run in under dt seconds. If it takes more time, set dt to a higher value.
  // Default is 0.010 seconds (10 miliseconds).
  dt = 0.010;

  // Customize PID constants. These variables are global, so you can optionally dynamically change them in your main task.
  gn_dth_dt = 0.23;
  gn_th = 25.00;
  gn_y = 272.8;
  gn_dy_dt = 24.6;
  kp = 0.0336;
  ki = 0.2688;
  kd = 0.000504;

  ///////////////////////////
  //END ADVANCED USER CONTROL
  ///////////////////////////

  //MOTOR SETUP
  nMotorPIDSpeedCtrl[motorB] = mtrNoReg;
  nMotorPIDSpeedCtrl[motorC] = mtrNoReg;
  nMotorEncoder[motorC] = 0;
  nMotorEncoder[motorB] = 0;

  //SENSOR SETUP
  int nSensorsDefined = 0;
	#ifdef HiTechnic_Gyro
	    SensorType[Gyro] = sensorRawValue;
	    // The following sets the average HiTechnic sensor value. If you know the average, you can avoid the calibration
	    // next time like so: mean_reading = 593.82; (if that's your sensor average).
	    mean_reading = calibrate_hitechnic();
	    nSensorsDefined++;
	#endif
	#ifdef DexterIndustries_dIMU
	    mean_reading = 0;
	    SensorType[Gyro] = sensorI2CCustomFastSkipStates;wait1Msec(200);
	    if(!DIMUconfigGyro(Gyro, DIMU_GYRO_RANGE_250,false)){
	      PlaySound(soundException);nxtDisplayTextLine(1,"dIMU Connected?");
	      wait1Msec(2000);StopAllTasks();}
	    wait1Msec(500);
	    nSensorsDefined++;
	#endif
	#ifdef MicroInfinity_Cruizcore
	  mean_reading = 0;
	  SensorType[Gyro] = sensorI2CCustom;
	  MICCreset(Gyro);
	  wait1Msec(500);
	  nSensorsDefined++;
	#endif
	#ifdef MindSensors_IMU
	  int   ux,uy,uz;									// Mindsensors Sensor Measurement
	  mean_reading = 0;
	  SensorType[Gyro] = sensorI2CCustomFastSkipStates;
	  wait1Msec(500);
		MSIMUsetGyroFilter(Gyro, 0x00);
		wait1Msec(1000);
	  nSensorsDefined++;
	#endif
	if(nSensorsDefined != 1){
	  nxtDisplayTextLine(0,"Check Sensor");
	  nxtDisplayTextLine(1,"definition!");
	  wait1Msec(5000);StopAllTasks();
	}

  //MATH CONSTANTS
  const float radius = your_wheel_diameter/1000;
  const float degtorad = PI/180;

  //SETUP VARIABLES FOR CALCULATIONS
  float u = 0;                    // Sensor Measurement (raw)
  float th = 0,//Theta            // Angle of robot (degree)
        dth_dt = 0;//dTheta/dt    // Angular velocity of robot (degree/sec)
  float e = 0,//Error             // Sum of four states to be kept zero: th, dth_dt, y, dy_dt.
        de_dt = 0,//dError/dt     // Change of above error
        _edt = 0,//Integral Error // Accumulated error in time
        e_prev = 0;//Previous Error/ Error found in previous loop cycle
  float pid = 0;                  // SUM OF PID CALCULATION
  float y = 0,//y                     // Measured Motor position (degrees)
        dy_dt = 0,//dy/dt             // Measured motor velocity (degrees/sec)
	      v = 0,//velocity          // Desired motor velocity (degrees/sec)
	      y_ref = 0;//reference pos // Desired motor position (degrees)
  int motorpower = 0,             // Power ultimately applied to motors
      last_steering = 0,          // Steering value in previous cycle
      straight = 0,               // Average motor position for synchronizing
      d_pwr = 0;                  // Change in power required for synchronizing
  const int n_max = 7;            // Number of measurement used for floating motor speed average
  int n = 0,n_comp = 0,           // Intermediate variables needed to compute measured motor speed
  encoder[n_max];                 // Array containing last n_max motor positions
  memset(&encoder[0],0,sizeof(encoder));
  starting_balancing_task = false;// We're done configuring. Main task now resumes.


  ClearTimer(T4);                 // This timer is used in the driver. Do not use it for other purposes!

  while(true)
  {

    //READ GYRO SENSOR
		#ifdef HiTechnic_Gyro
      u =   SensorRaw[Gyro];wait1Msec(2);
      u = u+SensorRaw[Gyro];
		#endif
		#ifdef DexterIndustries_dIMU
	  	u =   -1.68*DIMUreadGyroAxis(Gyro, DIMU_GYRO_Z_AXIS);
	  	u = u -1.68*DIMUreadGyroAxis(Gyro, DIMU_GYRO_Z_AXIS);
		#endif
		#ifdef MicroInfinity_Cruizcore
		  u = MICCreadTurnRate(Gyro)*0.030;
		#endif
		#ifdef MindSensors_IMU
			MSIMUreadGyroAxes(Gyro, ux, uy, uz);
			u = uz*0.0210;
		#endif
		////////////


		//COMPUTE GYRO ANGULAR VELOCITY AND ESTIMATE ANGLE
  	dth_dt = u/2 - mean_reading;
  	mean_reading = mean_reading*0.999 + (0.001*(dth_dt+mean_reading));
  	th = th + dth_dt*dt;

    //ADJUST REFERENCE POSITION ON SPEED AND ACCELERATION
    if(v < speed*10){
    v = v + acceleration*10*dt;}
    else if(v > speed*10){
    v = v - acceleration*10*dt;}
    y_ref = y_ref + v*dt;

  	//COMPUTE MOTOR ENCODER POSITION AND SPEED
  	n++;if(n == n_max){n = 0;}
  	encoder[n] = nMotorEncoder[motorB] + nMotorEncoder[motorC] + y_ref;
  	n_comp = n+1;if(n_comp == n_max){n_comp = 0;}
  	y = encoder[n]*degtorad*radius/gear_down_ratio;
  	dy_dt = (encoder[n] - encoder[n_comp])/(dt*(n_max-1))*degtorad*radius/gear_down_ratio;

  	//COMPUTE COMBINED ERROR AND PID VALUES
  	e = gn_th * th + gn_dth_dt * dth_dt + gn_y * y + gn_dy_dt * dy_dt;
  	de_dt = (e - e_prev)/dt;
  	_edt = _edt + e*dt;
  	e_prev = e;
  	pid = (kp*e + ki*_edt + kd*de_dt)/radius*gear_down_ratio;

  	//ADJUST MOTOR SPEED TO STEERING AND SYNCHING
    if(steering == 0){
        if(last_steering != 0){
	        straight = nMotorEncoder[motorC] - nMotorEncoder[motorB];}
		    d_pwr = (nMotorEncoder[motorC] - nMotorEncoder[motorB] - straight)/(radius*10/gear_down_ratio);}
    else{d_pwr = steering/(radius*10/gear_down_ratio);}
    last_steering = steering;

  	//CONTROL MOTOR POWER AND STEERING
  	motorpower = 	pid;
    motor[motorB] = motorpower + d_pwr;
    motor[motorC] = motorpower - d_pwr;

    //ERROR CHECKING OR SHUTDOWN
    if(abs(th)>60 || abs(motorpower) > 2000){
      StopAllTasks();}

    //WAIT THEN REPEAT
  	while(time1[T4] < dt*1000){
  	  wait1Msec(1);}
  	ClearTimer(T4);
  }
}

#ifdef HiTechnic_Gyro
int calibrate_hitechnic()
{
 //Function for finding the HiTechnic Gyro offset
 int mean_reading = 0, p = 0;

 while(nNxtButtonPressed == kEnterButton){}
 nxtDisplayTextLine(0,"Put Segway Down");
 wait1Msec(500);
 nxtDisplayTextLine(2,"Calibrating");
 nxtDisplayTextLine(3,"HiTechnic Gyro..");

 for(p = 0; p < 40;p++){
    mean_reading = mean_reading + SensorRaw[Gyro];
    wait1Msec(50);}
 mean_reading = mean_reading/40;
 PlayTone(500,50);

 if(mean_reading < 550 || mean_reading > 640){
    nxtDisplayTextLine(4,"FAILED. Exiting...");
    nxtDisplayTextLine(6,"Sensor Connected?");
    wait1Msec(2000);StopAllTasks();}

 eraseDisplay();
 nxtDisplayTextLine(2,"Done! Hold Segway");
 nxtDisplayTextLine(4,"upright and press");
 nxtDisplayTextLine(6,"the orange button");
 while(nNxtButtonPressed != kEnterButton){}
 while(nNxtButtonPressed == kEnterButton){}
 eraseDisplay();

 return(mean_reading);
}
#endif
