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

		const int max_iter = 3;
		int iter = 0;

		float refpos = 0;	// reference position
		const int sample_time = 20;	// sample time in milliseconds (ms)
		float dt = (sample_time - 2)/1000f;	// 
		float speed = 0;
		const int wheel_diameter = 55; // in millimeters (mm)
		int radius = wheel_diameter;

		const int max_index = 7;
		int[] enc_val = new int[max_index];
		int enc_index = 0;

		int nowOutOfBound = 0;
		int prevOutofBound = 0;
		int outOfBoundCount = 0;
		int outOfBound = 0;

		int ang = 0;
		int mean_ang = 0;
		int mean = 0;
		int steering = 0;
		int max_acceleration = 0;

		private EV3Brick ev3 = new EV3Brick();

		public GyroBoy ()
		{
		}

		public void start() {
			ev3.connect ();

			initialize ();
//			getBalancePos();
			control();
//			shutDown();

			ev3.disconnect ();
		}

		void control() {
			Console.WriteLine ("iter\trefpos\tdt\tspeed\tmotorpower\td_pwr\tmotorB\tmotorC");
			const float radius_const = 57.3f;
			float curr_err = 0;
			float acc_err = 0;
			float dif_err = 0;
			float prev_err = 0;

			while (iter++ < max_iter) {
				// Position
				refpos = refpos + (dt * speed * 0.002f);

				// ReadEncoders
				speed = getMotorSpeed ();
				float robot_speed = (radius * speed) / radius_const;
				float robot_position = (radius * (ev3.getMotorADegree () + ev3.getMotorDDegree ()) / 2) / radius_const;

				// ReadGyro

				// CombineSensorValues
				float input = gain_angle * ang
//				              + gain_angular_velocity * angular_velocity
				              + gain_motor_position * (robot_position - refpos)
				              + gain_motor_speed * robot_speed
					;

				// ReadConstants
//				Kp, Ki, Kd, dt

				// PID
//				input
				curr_err = input;
				acc_err += curr_err*dt;
				dif_err = (curr_err - prev_err) / dt;
				float output = curr_err * Kp
				               + acc_err * Ki
				               + dif_err * Kd;

				// Errors

				// GetSteer

				// SetMotorPower

				Console.WriteLine (iter + "\t" + refpos + "\t" + dt + "\t" + speed + "\t" + input + "\t" + output + "\t"); 
//				Console.WriteLine (iter + "\t" + u + "\t" + pid + "\t" + th + "\t" + motorpower 
//					+ "\t" + d_pwr + "\t" + motor[motorB] + "\t" + motor[motorC]);

				// Wait
			}
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
		int calibrate() {
			Console.WriteLine ("calibrating ...");

			// Play tone: frequency 440Hz, volume 10
			// duration 0.1sec, play type 0
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
			// duration 0.1sec, play type 0
			ev3.sound (10, 440, (int)(0.1 * 1000));

			Thread.Sleep (100);
			// Play tone
			ev3.sound (10, 440, (int)(0.1 * 1000));

			return mean;
		}

		/**
		 * average of 5 samples
		 */
		int gyroRate() {
			int filter = 0;

			// get 5 samples
			for(int i = 0; i < 5; i ++)
				filter = ev3.getAngularVelocity () + filter;

			return filter / 5;
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
	}
}

