using System;

namespace ev3
{
	/**
	 * Segway using PID control
	 */
	public class Segway
	{
		int steering = 0;
		int acceleration = 50;
		int speed = 0;
		bool starting_balancing_task = true;
		const float wheel_diameter = 0;
		EV3Brick ev3 = new EV3Brick();

		//GLOBAL VARIABLE SETUP
		float gn_dth_dt,gn_th,gn_y,gn_dy_dt,kp,ki,kd,mean_reading,gear_down_ratio,dt;

		public Segway ()
		{
		}

		public void start() {
			ev3.connect();

			balance();
			while (starting_balancing_task) {}

			ev3.disconnect ();
			Console.WriteLine("End");
		}

		private void balance() {
			Console.WriteLine("start balancing ...");
			starting_balancing_task = true;

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
			dt = 0.010f;

			// Customize PID constants. These variables are global, so you can optionally dynamically change them in your main task.
			gn_dth_dt = 0.23f;
			gn_th = 25.00f;
			gn_y = 272.8f;
			gn_dy_dt = 24.6f;
			kp = 0.0336f;
			ki = 0.2688f;
			kd = 0.000504f;

			///////////////////////////
			//END ADVANCED USER CONTROL
			///////////////////////////

			//MOTOR SETUP
			int motorB = 1;
			int motorC = 2;
			int mtrNoReg = 0;
			int[] nMotorPIDSpeedCtrl = new int[4];
			nMotorPIDSpeedCtrl[motorB] = mtrNoReg;
			nMotorPIDSpeedCtrl[motorC] = mtrNoReg;
			int[] nMotorEncoder = new int[4];
			nMotorEncoder[motorC] = 0;
			nMotorEncoder[motorB] = 0;
			int[] motor = new int[4];

			// Sensor setup
			int Gyro = 2;
			int[] SensorType = new int[5];
//			SensorType[Gyro] = ev3.getAngularVelocity();

			int nSensorsDefined = 0;
			/*
	#ifdef HiTechnic_Gyro
	    SensorType[Gyro] = sensorRawValue;
	    // The following sets the average HiTechnic sensor value. If you know the average, you can avoid the calibration
	    // next time like so: mean_reading = 593.82; (if that's your sensor average).
	    mean_reading = calibrate_hitechnic();
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
*/
			//MATH CONSTANTS
			const float radius = wheel_diameter/1000;
			const float degtorad = (float)Math.PI/180;

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
			int n = 0, n_comp = 0;           // Intermediate variables needed to compute measured motor speed
			int[] encoder = new int[n_max];                 // Array containing last n_max motor positions
//			memset(&encoder[0],0,sizeof(encoder));
			starting_balancing_task = false;// We're done configuring. Main task now resumes.

//	ClearTimer(T4);                 // This timer is used in the driver. Do not use it for other purposes!

			while(true)
			{

			//READ GYRO SENSOR
						/*
				#ifdef HiTechnic_Gyro
			  u =   SensorRaw[Gyro];wait1Msec(2);
			  u = u+SensorRaw[Gyro];
				#endif
				#ifdef MindSensors_IMU
					MSIMUreadGyroAxes(Gyro, ux, uy, uz);
					u = uz*0.0210;
				#endif
				////////////
		*/

				//COMPUTE GYRO ANGULAR VELOCITY AND ESTIMATE ANGLE
				dth_dt = u/2 - mean_reading;
				mean_reading = mean_reading*0.999f + (0.001f*(dth_dt+mean_reading));
				th = th + dth_dt*dt;

				//ADJUST REFERENCE POSITION ON SPEED AND ACCELERATION
				if(v < speed*10){
					v = v + acceleration*10*dt;
				} else if(v > speed*10){
					v = v - acceleration*10*dt;
				}
				y_ref = y_ref + v*dt;

				//COMPUTE MOTOR ENCODER POSITION AND SPEED
				n++;if(n == n_max){n = 0;}
				encoder[n] = nMotorEncoder[motorB] + nMotorEncoder[motorC] + (int)y_ref;
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
				        straight = nMotorEncoder[motorC] - nMotorEncoder[motorB];
					}
					d_pwr = (int)((nMotorEncoder[motorC] - nMotorEncoder[motorB] - straight)/(radius*10/gear_down_ratio));
				} else{d_pwr = (int)(steering/(radius*10/gear_down_ratio));}
				last_steering = steering;

				//CONTROL MOTOR POWER AND STEERING
				motorpower = 	(int)pid;
				motor[motorB] = motorpower + d_pwr;
				motor[motorC] = motorpower - d_pwr;

				//ERROR CHECKING OR SHUTDOWN
				Console.WriteLine (u + " " + pid + " " + th + " " + motorpower);
				if(pid.Equals(float.NaN) || Math.Abs(th)>60 || Math.Abs(motorpower) > 2000){
//				  StopAllTasks();
					Console.WriteLine ("error");
					break;
				}

				ev3.onMotorA (motor [motorB]);
				ev3.onMotorD (motor [motorC]);

//				//WAIT THEN REPEAT
//				while(time1[T4] < dt*1000){
//				  wait1Msec(1);
//				}
//				ClearTimer(T4);
			}

//			starting_balancing_task = false;
		}
	}
}

