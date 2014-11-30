import lejos.hardware.Button;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.EncoderMotor;
import lejos.utility.Stopwatch;

/**
 * Segway using PID control in RobotC
 */
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
                          - Both supported sensors now use same PID private static finalants
                          - Added section at the beginning of the code for user adjustable settings
- v0.97 07 March    2012: - Loop cycle now uses timer to determine cycle length.
                          - Motor encoder speed is now really defined as degrees per second.
                          - Both changes further increase flexibility of the driver, but
                            the consequence is that the private static finalants have been adjusted to
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

public class Segway
{
	private static final boolean DEBUG = false;
	private static final int max_iter = 10000;

	EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2);
	EncoderMotor leftMotor = new NXTMotor(MotorPort.A); 
	EncoderMotor rightMotor = new NXTMotor(MotorPort.D); 
	
	private static final float wheel_diameter = 56;	// in millimeters
	
	private static final int SPEED = 30;	// default speed
	private static final int DRIVE = 7;	// default steering -7
	private static final int SAMPLE_SIZE = 2;
	
	//GLOBAL VARIABLE SETUP
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
	private final static float gear_down_ratio = 1f; // 0.1/2 bad

	// Set the time each loop cycle should last. You can set it up to 0.03 seconds or even higher, if you really
	// need to. If you add code to the control loop below (such as to read another sensor), make sure that
	// all code can run in under dt seconds. If it takes more time, set dt to a higher value.
	// Default is 0.010 seconds (10 miliseconds).
	private final static float dt = 0.010f; // 0.003/0.005 good, 0.010 not bad, 0.02 not good

	// Customize PID private static finalants. These variables are global, so you can optionally dynamically change them in your main task.
	// (0.01, 0.05, 0.0001) good
	// (0.01, 0.05, 0.0005) no good
	private final static float kp = 0.01f;		// [0.01,0.1], default 0.0336
	private final static float ki = 0.05f;		// [0.001, 0.01] default 0.2688
	private final static float kd = 0.0001f;		// default 0.000504
	private final static float gn_dth_dt = 0.23f;	// default 0.23f
	private final static float gn_th = 25.00f;		// default 25
	private final static float gn_y = 272.8f;		// default 272.8
	private final static float gn_dy_dt = 24.6f;	// default 24.6

	// from GyroBoy: not working
//	private static final float kp = 0.5f;  // default 0.5f
//	private static final float ki = 11;   // default 11
//	private static final float kd = 0.005f; // default 0.005f
//	private static final float gn_dth_dt = 1.3f; // for theta_hat
//	private static final float gn_th = 25;		// for theta
//	private static final float gn_dy_dt = 75;	// for y_hat, default 75
//	private static final float gn_y = 350;	// for y, default 350
	
//	float gn_dth_dt,gn_th,gn_y,gn_dy_dt,kp,ki,kd,mean_reading,gear_down_ratio,dt;
	float mean_reading;
	Stopwatch stopwatch = new Stopwatch();
	
	///////////////////////////
	//END ADVANCED USER CONTROL
	///////////////////////////
	//MOTOR SETUP
	private final static int motorA = 1;
	private final static int motorD = 2;
//	private final static int mtrNoReg = 0;
//	private final static int[] nMotorPIDSpeedCtrl = new int[4];
	private final static int[] nMotorEncoder = new int[4];
	int[] motor = new int[4];

	//MATH private static finalANTS
	private static final float radius = wheel_diameter/1000;
	private static final float degtorad = (float)Math.PI/180;

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
	private static final int n_max = 7;            // Number of measurement used for floating motor speed average
	int n = 0, n_comp = 0;           // Intermediate variables needed to compute measured motor speed
	int[] encoder = new int[n_max];                 // Array containing last n_max motor positions

	int steering = 0;
	int acceleration = 50;
	int speed = 0;
	boolean starting_balancing_task = true;
	
	public static void main (String[] args) {
		Segway segway = new Segway();
		segway.start();
	}
	
	public void start() {
		initialize();
		
		Thread t1 = new Thread(new Balancer());
		t1.start ();
//		while (starting_balancing_task) {}
		System.out.println ("Complete balancing task.");

		steering = DRIVE;
		speed = SPEED;

		if(Button.ESCAPE.isDown())
			stopMotors();
		
		try {
			t1.join ();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		stopMotors();
		
		try {
			Thread.sleep(10000);
		} catch (Exception e) {
		}
	}

	void initialize() {
		starting_balancing_task = true;

		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		
		nMotorEncoder[motorD] = 0;
		nMotorEncoder[motorA] = 0;

		// Sensor setup
		gyro.reset();
		mean_reading = 0;
		
		starting_balancing_task = false;// We're done configuring. Main task now resumes.
	}
	 
	class Balancer implements Runnable{

		//		private void balance() {
		public void run() {
			System.out.println("start balancing ...");

			//	ClearTimer(T4);                 // This timer is used in the driver. Do not use it for other purposes!
			stopwatch.reset();
			
//			System.out.println ("iter\tu\tpid\tth\tmotorpower\td_pwr\tmotorA\tmotorD");

			int iter = 0;
			//			while(true)
			while(iter ++ < max_iter || !Button.ESCAPE.isDown())
			{
				//READ GYRO SENSOR
//				u = ev3.getAngularVelocity ();
				u = gyroRate(SAMPLE_SIZE);
//				try {
//					Thread.sleep (2);
//				} catch (InterruptedException e1) {
//					e1.printStackTrace();
//				}
//				u += gyroRate();

				//COMPUTE GYRO ANGULAR VELOCITY AND ESTIMATE ANGLE
//				dth_dt = u/2 - mean_reading;
				dth_dt = u - mean_reading;
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
				nMotorEncoder [motorA] = leftMotor.getTachoCount();
				nMotorEncoder [motorD] = rightMotor.getTachoCount();
				n++;if(n == n_max){n = 0;}
				encoder[n] = nMotorEncoder[motorA] + nMotorEncoder[motorD] + (int)y_ref;
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
						straight = nMotorEncoder[motorD] - nMotorEncoder[motorA];
					}
					d_pwr = (int)((nMotorEncoder[motorD] - nMotorEncoder[motorA] - straight)/(radius*10/gear_down_ratio));
				} else{d_pwr = (int)(steering/(radius*10/gear_down_ratio));}
				last_steering = steering;

				//CONTROL MOTOR POWER AND STEERING
				motorpower = 	(int)pid;
				motor[motorA] = motorpower + d_pwr;
				motor[motorD] = motorpower + d_pwr;
//				motor[motorD] = motorpower - d_pwr;

				//ERROR CHECKING OR SHUTDOWN
//				System.out.println (iter + "\t" + u + "\t" + pid + "\t" + th + "\t" + motorpower 
//						+ "\t" + d_pwr + "\t" + motor[motorA] + "\t" + motor[motorD]);
//				if(pid.Equals(float.NaN) || Math.Abs(th)>60 || Math.Abs(motorpower) > 2000){
				if(Math.abs(th)>60 || Math.abs(motorpower) > 5000){
					//				  StopAllTasks();
					System.out.println ("Error");
					System.out.printf ("%d %.2f %.2f %.0f %d\n", iter, u, th, pid, motorpower);
//					System.out.printf ("%d %.2f %.2f %.0f %d\n", iter, u, th, pid, motorpower 
//							+ "\t" + d_pwr + "\t" + motor[motorA] + "\t" + motor[motorD]
//									);
					stopMotors ();
					break;
				} else {
					if(DEBUG)
						System.out.printf ("%d %.2f %.2f %.0f %d\n", iter, u, th, pid, motorpower);
				}
				
				leftMotor.setPower(motor[motorA]);
				rightMotor.setPower(motor[motorD]);
				
				//				//WAIT THEN REPEAT
				//				while(time1[T4] < dt*1000){
				//				  wait1Msec(1);
				//				}
				//				ClearTimer(T4);
				while(stopwatch.elapsed() < dt*1000) {
					try {
						Thread.sleep (1);
					} catch (InterruptedException e1) {
						e1.printStackTrace();
					}
				}
				stopwatch.reset();
			}
		}
	}

	/**
	 * average of angular velocity samples
	 */
	float gyroRate(int sample_size) {
		float filter = 0;

		// get samples
//		int sample_size = gyro.getAngleMode().sampleSize();
		float[] sample = new float[1];
		for(int i = 0; i < sample_size; i ++) {
			gyro.getRateMode().fetchSample(sample, 0);
			filter += sample[0];
		}
	
		return filter / sample_size;
	}

	void stopMotors() {
		leftMotor.flt();
		rightMotor.flt();
	}
}
