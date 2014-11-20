using System;
using System.Threading;
using System.Timers;

namespace ev3
{
	public class GyroBoy
	{
		const float Kp = 0.5f;
		const float Ki = 11;
		const float Kd = 0.005f;

		const float gain_angular_velocity = 1.3f; // for theta_hat
		const float gain_angle = 25;		// for theta
		const float gain_motor_speed = 75;	// for y_hat
		const float gain_motor_position = 350;	// for y

		const int max_iter = 20;
		const int drive_sleep = 7000; // milliseconds

//		bool sound = false;
		bool sound = true;

		float refpos = 0;	// reference position
		const int sample_time = 22;	// sample time in milliseconds (ms)
		const float dt = (sample_time - 2)/1000f;	// 
		float speed = 0;
		const int wheel_diameter = 55; // in millimeters (mm)
		float radius = wheel_diameter / 2000f;

		const int max_index = 7;
		int[] enc_val = new int[max_index];
		int enc_index = 0;

		bool nowOutOfBound = false;
		bool prevOutOfBound = false;
		int outOfBoundCount = 0;
		int outOfBound = 0;

		float ang = 0;
		float mean_ang = 0;
		float mean = 0;
		int steering = 0;
		int max_acceleration = 0;

		const string FORMAT = "0.00"; // precision

		bool complete = false;

		private EV3Brick ev3 = new EV3Brick();

		public GyroBoy ()
		{
		}

		public void start() {
			ev3.connect ();

			initialize ();
//			balance();
			Thread t1 = new Thread (new ThreadStart (balance));
			t1.Start ();
			Thread t2 = new Thread (new ThreadStart (drive));
			t2.Start ();

			t1.Join ();
			t2.Join ();

			ev3.disconnect ();
		}

		void drive() {
			Console.WriteLine ("driving ...");

			int iter = 0;

//			while (true) {
			while (!complete) {
				Thread.Sleep (drive_sleep);
				speed = 20;
				Thread.Sleep (drive_sleep);
				speed = -20;
			}
		}

		void balance() {
			Console.WriteLine("refpos = " + refpos + ", dt = " + dt + ", Kp = " + Kp + ", Ki = " + Ki + ", Kd = " + Kd);
			Console.WriteLine ("iter\tspeed\tang_vel\tang"
				+ "\tsensor\tavg_pwr\toffset\trefpos"
				+ "\trspeed"
				+ "\tspeedA\tspeedD\textra\tpwr_b\tpwr_c"
//				+ "\tcurr_err\tacc_err\tdif_err\tprev_err"
//				+ "\tmotorB\tmotorC" 
			);

			int iter = 0;

			const float radius_const = 57.3f;
			float curr_err = 0;
			float acc_err = 0;
			float dif_err = 0;
			float prev_err = 0;

			while (iter++ < max_iter) {
				// Position
				refpos = refpos + (dt * speed * 0.002f);

				// ReadEncoders
//				speed = getMotorSpeed ();
				float robot_speed = (radius * getMotorSpeed ()) / radius_const;
				float robot_position = (radius * (ev3.getMotorADegree () + ev3.getMotorDDegree ()) / 2) / radius_const;

				// ReadGyro
				float ang_vel = readGyro ();

				// CombineSensorValues
				float sensor_values = gain_angle * ang
				              + gain_angular_velocity * ang_vel
				              + gain_motor_position * (robot_position - refpos)
				              + gain_motor_speed * robot_speed
					;

				// PID
				// input: sensor values
				// output: average power
				float avg_pwr = pid (sensor_values, curr_err, acc_err, dif_err, prev_err);

				// Errors
				// input: PID output
				errors (avg_pwr);

				Console.Write (iter + "\t" + speed + "\t" + ang_vel.ToString(FORMAT) + "\t" + ang.ToString(FORMAT)
					+ "\t" + sensor_values.ToString(FORMAT) + "\t" + avg_pwr.ToString(FORMAT) 
					+ "\t" + (robot_position - refpos).ToString(FORMAT) + "\t" + refpos.ToString(FORMAT)
					+ "\t" + robot_speed.ToString(FORMAT) + "\t"
				); 

				// SetMotorPower
				// input: pid output
				setMotorPower (avg_pwr);

				// Wait
				// Timer >= dt, elapsedTime
				// timer.reset();
			}

			complete = true;
		}

		void initialize() {
			Console.WriteLine ("initializing ...");
//			dt = (sample_time - 2)/1000;

			// erase array
			for (int i = 0; i < max_index; i++) {
				enc_val [i] = 0;
			}

			ev3.resetMotorATachoCount ();
			ev3.resetMotorDTachoCount ();

			Thread.Sleep (100);

			mean = calibrate ();

			// reset timer 
			System.Timers.Timer timer = new System.Timers.Timer ();
//			timer.AutoReset ();
		}

		/**
		 * average of 20 gyroRate values
		 */
		float calibrate() {
			Console.WriteLine ("calibrating ...");

			// Play tone: frequency 440Hz, volume 10
			// duration 0.1sec, play type 0
			if(sound)
				ev3.sound (10, 440, (int)(0.1 * 1000));

			Thread.Sleep (100);
			mean = 0;

			int count = 1;

			// gyro rate
			for (int i = 0; i < count; i++) {
				mean += gyroRate ();
				Console.WriteLine ("gyroRate mean: " + mean);
				Thread.Sleep (5);
			}
			mean = mean / count;

			Thread.Sleep (100);
			// Play tone: frequency 440Hz, volume 10
			if(sound)
				ev3.sound (10, 440, (int)(0.1 * 1000));

			Thread.Sleep (100);
			// Play tone
			if(sound)
				ev3.sound (10, 440, (int)(0.1 * 1000));

			return mean;
		}

		/**
		 * average of 5 samples
		 */
		float gyroRate() {
			float filter = 0;

			// get 5 samples
			for(int i = 0; i < 5; i ++)
				filter = ev3.getAngularVelocity () + filter;

			return filter / 5f;
		}

		float readGyro() {
			float curr_val = gyroRate ();
			mean = mean * (1f - 0.2f * dt) + (curr_val * 0.2f * dt);
			float ang_vel = curr_val - mean;
			ang = ang + dt * ang_vel;
			mean_ang = mean_ang * 0.999f + ang * (1f - 0.999f);
			ang = ang - mean_ang;

			return ang_vel;
		}

		int getMotorSpeed() {
			enc_index++;

			if (max_index <= enc_index)
				enc_index = 0;

			int compare_index = enc_index + 1;
			if (max_index <= compare_index)
				compare_index = 0;

			enc_val[enc_index] = (ev3.getMotorADegree() + ev3.getMotorDDegree())/2;
//			Console.WriteLine (enc_val [enc_index] + " " + enc_val[compare_index] + " " + max_index + " " + dt);

			return (int)((enc_val [enc_index] - enc_val [compare_index]) / (max_index * dt));
		}

		public float pid(float sensor_values, float curr_err, float acc_err, float dif_err, float prev_err) {
			const float ref_val = 0;

			curr_err = sensor_values - ref_val;
			acc_err += curr_err*dt;
			dif_err = (curr_err - prev_err) / dt;

			return curr_err * Kp
				+ acc_err * Ki
				+ dif_err * Kd;
		}

		public void setMotorPower(float avg_pwr) {
			// limit: [-50, 50]
			if (avg_pwr > 50)
				avg_pwr = 50;
			if (avg_pwr < -50)
				avg_pwr = -50;

			float new_steering = avg_pwr;
			float old_steering = 0;
			float extra_pwr = 0;
			if (new_steering == 0) {
				int sync_0 = ev3.getMotorDDegree() - ev3.getMotorADegree();

				if (old_steering != 0) {
					extra_pwr = (ev3.getMotorDDegree () - ev3.getMotorADegree () - sync_0) * 0.05f;
				}
			} else {
				extra_pwr = new_steering * (-0.5f);
			}

			float pwr_c = extra_pwr - avg_pwr;
			float pwr_b = extra_pwr + avg_pwr;
			old_steering = new_steering;

			float speedA = (pwr_b * 0.021f / radius);
			float speedD = (pwr_c * 0.021f / radius);
//			ev3.setPowerMotorA ((int)speedA);
//			ev3.setPowerMotorD ((int)speedD);
			ev3.onMotorA ((int)speedA);
			ev3.onMotorD ((int)speedD);
			Console.WriteLine (speedA.ToString(FORMAT) + "\t" + speedD.ToString(FORMAT) 
				+ "\t" + extra_pwr.ToString(FORMAT) + "\t" + pwr_b.ToString(FORMAT) + "\t" + pwr_c.ToString(FORMAT));
//			Console.WriteLine ("speedA = " + speedA + ", speedD = " + speedD
//				+ ", extra_pwr = " + extra_pwr + ", pwr_b = " + pwr_b + ", pwr_c = " + pwr_c
//			);
		}

		public void errors(float avg_pwr) {
			nowOutOfBound = (Math.Abs (avg_pwr) > 100);

			if (nowOutOfBound && prevOutOfBound) {
				outOfBound++;
			} else {
				outOfBound = 0;
			}

			if (outOfBound > 20) {
				Thread.Sleep (100);
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

				Thread.Sleep (4000);
				// stop
			} else {
				prevOutOfBound = nowOutOfBound;
			}
		}
	}
}

