import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Stopwatch;

public class GyroBoy
{
	private static final float Kp = 0.5f;
	private static final float Ki = 11;
	private static final float Kd = 0.005f;

	private static final float gain_angular_velocity = 1.3f; // for theta_hat
	private static final float gain_angle = 25;		// for theta
	private static final float gain_motor_speed = 75;	// for y_hat
	private static final float gain_motor_position = 350;	// for y

	private static final int max_iter = 10;
	private static final int drive_sleep = 7000; // milliseconds

	//boolean sound = false;
	boolean sound = true;

	float refpos = 0;	// reference position
	private static final int sample_time = 22;	// sample time in milliseconds (ms)
	private static final float dt = (sample_time - 2)/1000f;	// verified
	float speed = 0;
	private static final int wheel_diameter = 55; // in millimeters (mm)
	private static final float radius = wheel_diameter / 2000f; // verified

	private static final int max_index = 7;
	float[] enc_val = new float[max_index];
	int enc_index = 0;

	boolean nowOutOfBound = false;
	boolean prevOutOfBound = false;
	int outOfBoundCount = 0;
	//		int outOfBound = 0;

	float ang = 0;
	//		float mean_ang = 0;
	float mean = 0;
	int steering = 0;
	float old_steering = 0;
	int max_acceleration = 0;

	private static final float radius_const = 57.3f;
	float acc_err = 0;
	float prev_err = 0;

	private static final String FORMAT = "0.00"; // precision

	boolean complete = false;

	Stopwatch stopwatch = new Stopwatch();

//	private EV3Brick ev3 = new EV3Brick();
	EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2);
    private DifferentialPilot pilot = new DifferentialPilot(5.6, 9.25, Motor.A, Motor.D);
	
	public GyroBoy ()
	{
	}

	// verified
	public void start() {
//		ev3.connect ();

		// verified
		initialize ();

		// verified
		Thread t1 = new Thread (new Balancer());
		t1.start ();
		// verified
		Thread t2 = new Thread (new Driver());
		t2.start ();

		try {
			t1.join ();
			t2.join ();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

//		ev3.disconnect ();
	}

	// verified
	// controls speed and steering
	class Driver implements Runnable {
		public void run() {
			System.out.println ("driving ...");

			//			while (true) {
			while (!complete) {
				speed = 0;
				sleep (drive_sleep);
				speed = -20;
				sleep (drive_sleep);
				speed = 20;
				sleep (drive_sleep);
			}
		}
	}

	// verified
	class Balancer implements Runnable{
		public void run() {
			System.out.println("refpos = " + refpos + ", dt = " + dt + ", Kp = " + Kp + ", Ki = " + Ki + ", Kd = " + Kd);
			System.out.println ("iter\tspeed\tang_vel\tang"
					+ "\tsensor\tavg_pwr\toffset\trefpos"
					+ "\tmspeed"
					+ "\tspeedA\tspeedD\textra\tpwr_b\tpwr_c"
					//				+ "\tcurr_err\tacc_err\tdif_err\tprev_err"
					);

			Stopwatch totalwatch = new Stopwatch();
			Stopwatch functionwatch = new Stopwatch();
			int iter = 0;

			stopwatch.reset();

			while (iter++ < max_iter) {
				// Position: verified
				refpos = refpos + (dt * speed * 0.002f);

				// ReadEncoders: verified
				//functionwatch.Restart();
				float motor_speed = (radius * getMotorSpeed ()) / radius_const;
//				float motor_position = (radius * (ev3.getMotorADegree () + ev3.getMotorDDegree ()) / 2.0f) / radius_const;
				float motor_position = (radius * (pilot.getAngleIncrement())) / radius_const;
//				float motor_position = (radius * (pilot.getMovement().getArcRadius()) / radius_const;
				//Console.Write(functionwatch.ElapsedMilliseconds + "ms ");

				// ReadGyro: verified
				//functionwatch.Restart();
				float ang_vel = readGyro();
				//Console.Write(functionwatch.ElapsedMilliseconds + "ms ");

				// CombineSensorValues: verified
				//functionwatch.Restart();
				float sensor_values = combineSensorValues (ang_vel, motor_position, motor_speed);
				//Console.Write(functionwatch.ElapsedMilliseconds + "ms ");

				// PID: verified
				// input: sensor values
				// output: average power
				//functionwatch.Restart();
				float avg_pwr = pid (sensor_values);
				//				float avg_pwr = pid (sensor_values, curr_err, acc_err, dif_err, prev_err);
				//Console.Write(functionwatch.ElapsedMilliseconds + "ms ");

				// Errors: verified
				// input: PID output
				//functionwatch.Restart();
				errors (avg_pwr);
				//Console.Write(functionwatch.ElapsedMilliseconds + "ms ");

				/*
				Console.Write (iter + "\t" + speed + "\t" + ang_vel.ToString(FORMAT) + "\t" + ang.ToString(FORMAT)
					+ "\t" + sensor_values.ToString(FORMAT) + "\t" + avg_pwr.ToString(FORMAT) 
					+ "\t" + (motor_position - refpos).ToString(FORMAT) + "\t" + refpos.ToString(FORMAT)
					+ "\t" + motor_speed.ToString(FORMAT) + "\t"
				); 
				 */

				// SetMotorPower: verified
				// input: pid output
				//functionwatch.Restart();
				setMotorPower(avg_pwr);
				//System.out.println(functionwatch.ElapsedMilliseconds + "ms");

				// Wait: verified
				// Timer >= dt, elapsedTime
				int elapsedTime = stopwatch.elapsed();
				System.out.println(elapsedTime + " " + totalwatch.elapsed());
				if(elapsedTime >= dt * 1000f)
					stopwatch.reset();
			}

			complete = true;
		}
	}

	// verified
	void initialize() {
		System.out.println ("initializing ...");
		//			dt = (sample_time - 2)/1000;

		// erase array
		for (int i = 0; i < max_index; i++) {
			enc_val [i] = 0;
		}

//		stopwatch = Stopwatch.StartNew();
//		ev3.resetMotorATachoCount ();
//		ev3.resetMotorDTachoCount();
		pilot.reset();
		System.out.println("Reset tacho count: " + (stopwatch.elapsed()/2f) + "ms");

		sleep (100);

		// verified
		stopwatch.reset();
		mean = calibrate ();
		System.out.println(stopwatch.elapsed() + "ms");

		speed = 0;
		steering = 0;

		// reset timer 
		stopwatch.reset();
	}

	/**
	 * verified
	 * average of 20 gyroRate values
	 */
	float calibrate() {
		System.out.println ("calibrating ...");

		// Play tone: frequency 440Hz, volume 10
		// duration 0.1sec, play type 0
		if(sound)
			Sound.playTone(440, 100, 10);

		sleep (100);
		mean = 0;

		int count = 20;

		// gyro rate
		for (int i = 0; i < count; i++) {
			mean += gyroRate ();
			//System.out.println ("gyroRate mean: " + mean);
			sleep (5);
		}
		mean = mean / count;

		//System.out.println ("gyroRate mean: " + mean);

		sleep (100);
		// Play tone: frequency 440Hz, volume 10
		if(sound)
			Sound.playTone(440, 100, 10);

		sleep (100);
		// Play tone
		if(sound)
			Sound.playTone(440, 100, 10);

		return mean;
	}

	/**
	 * verified
	 * average of 5 samples
	 */
	float gyroRate() {
		float filter = 0;

		// get 5 samples
		for(int i = 0; i < 5; i ++)
			filter = ev3.getAngularVelocity () + filter;

		return filter / 5f;
	}

	// verified
	// change ang and return ang_vel
	float readGyro() {
		Stopwatch gyrowatch = Stopwatch.StartNew();
		float curr_val = gyroRate ();
		Console.Write("gyro Rate: " + (gyrowatch.ElapsedMilliseconds/5f).ToString(FORMAT) + "ms ");

		// EMA
		mean = mean * (1f - 0.2f * dt) + (curr_val * 0.2f * dt);
		float ang_vel = curr_val - mean;
		ang = ang + dt * ang_vel;

		// what is this part? in lighter color
		//			mean_ang = mean_ang * 0.999f + ang * (1f - 0.999f);
		//			ang = ang - mean_ang;

		return ang_vel;
	}

	// verified
	float getMotorSpeed() {
		enc_index++;

		if (max_index <= enc_index)
			enc_index = 0;

		int compare_index = enc_index + 1;
		if (max_index <= compare_index)
			compare_index = 0;

		enc_val[enc_index] = (ev3.getMotorADegree() + ev3.getMotorDDegree())/2.0f;
		//			System.out.println (enc_val [enc_index] + " " + enc_val[compare_index] + " " + max_index + " " + dt);

		return ((enc_val [enc_index] - enc_val [compare_index]) / (max_index * dt));
	}

	// verified
	public float combineSensorValues(float ang_vel, float motor_position, float motor_speed) {
		return gain_angle * ang
				+ gain_angular_velocity * ang_vel
				+ gain_motor_position * (motor_position - refpos)
				+ gain_motor_speed * motor_speed;
	}

	// verified, but missing prev_err = curr_err
	//		public float pid(float sensor_values, float curr_err, float acc_err, float dif_err, float prev_err) {
	public float pid(float sensor_values) {
		private static final float ref_val = 0;

		float curr_err = sensor_values - ref_val;
		acc_err += curr_err * dt;
		float dif_err = (curr_err - prev_err) / dt;
		prev_err = curr_err;

		return curr_err * Kp
				+ acc_err * Ki
				+ dif_err * Kd;
	}

	// verified
	// read the shared variable steering
	public void setMotorPower(float avg_pwr) {
		// limit steering: [-50, 50]
		float new_steering = steering;
		if (steering > 50)
			new_steering = 50;
		if (steering < -50)
			new_steering = -50;

		float extra_pwr = 0;
		if (new_steering == 0) {
			int sync_0 = 0;

			if (old_steering != 0) {
				sync_0 = ev3.getMotorDDegree() - ev3.getMotorADegree();
			}
			extra_pwr = (ev3.getMotorDDegree () - ev3.getMotorADegree () - sync_0) * 0.05f;
		} else {
			extra_pwr = new_steering * (-0.5f);
		}

		float pwr_c = avg_pwr - extra_pwr;
		float pwr_b = avg_pwr + extra_pwr;
		old_steering = new_steering;

		float speedA = (pwr_b * 0.021f / radius);
		float speedD = (pwr_c * 0.021f / radius);
		//			ev3.setPowerMotorA ((int)speedA);
		//			ev3.setPowerMotorD ((int)speedD);
		ev3.onMotorA ((int)speedA);
		ev3.onMotorD ((int)speedD);
		//System.out.println (speedA.ToString(FORMAT) + "\t" + speedD.ToString(FORMAT) 
		//	+ "\t" + extra_pwr.ToString(FORMAT) + "\t" + pwr_b.ToString(FORMAT) + "\t" + pwr_c.ToString(FORMAT));
	}

	// verified except the interrupting balance loop
	public void errors(float avg_pwr) {
		nowOutOfBound = (Math.abs (avg_pwr) > 100f);

		// read cur_err

		if (nowOutOfBound && prevOutOfBound) {
			outOfBoundCount++;
		} else {
			outOfBoundCount = 0;
		}

		if (outOfBoundCount > 20) {
			sleep (100);
			ev3.offMotorA ();
			ev3.offMotorD ();

			// diplay ERROR

			ev3.sound (50, 800, 100);
			ev3.sound (50, 600, 100);
			ev3.sound (50, 300, 100);

			// B+C
			//				ev3.offMotorB ();
			//				ev3.offMotorC ();

			// interrupt balance loop

			sleep (4000);
			// stop
		} else {
			prevOutOfBound = nowOutOfBound;
		}
	}
	
	private void sleep(int milliseconds) {
		try {
			Thread.sleep(milliseconds);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}

