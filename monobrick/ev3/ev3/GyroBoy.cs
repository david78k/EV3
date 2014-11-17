using System;

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

		const int max_iter = 10;
		int iter = 0;

		float refpos = 0;	// reference position
		const int sample_time = 20;	// sample time in milliseconds (ms)
		float dt = (sample_time - 2)/1000;	// 
		float speed = 0;
		const int wheel_diameter = 55; // in millimeters (mm)
		int radius = wheel_diameter;

		const int max_index = 7;
		int[] enc_val = new int[max_index];

		private EV3Brick ev3 = new EV3Brick();

		public GyroBoy ()
		{
		}

		public void start() {
//			ev3.connect ();

//			initialize ();
//			getBalancePos();
			control();
//			shutDown();

//			ev3.disconnect ();
		}

		void control() {
			Console.WriteLine ("iter\trefpos\tdt\tspeed\tmotorpower\td_pwr\tmotorB\tmotorC");

			while (iter++ < max_iter) {
				// Position
				refpos = refpos + (dt * speed * 0.002f);

				// ReadEncoders

				// ReadGyro

				// CombineSensorValues

				// ReadConstants

				// PID

				// Errors

				//GetSteer

				// SetMotorPower

				Console.WriteLine (iter + "\t" + refpos + "\t" + dt + "\t" + speed + "\t"); 
//				Console.WriteLine (iter + "\t" + u + "\t" + pid + "\t" + th + "\t" + motorpower 
//					+ "\t" + d_pwr + "\t" + motor[motorB] + "\t" + motor[motorC]);

				// Wait
			}
		}

		void initialize() {
//			dt = (sample_time - 2)/1000;

			// erase array
			for (int i = 0; i < max_index; i++) {
				enc_val [i] = 0;
			}

			ev3.resetMotorATachoCount ();
			ev3.resetMotorDTachoCount ();
		}

	}
}

