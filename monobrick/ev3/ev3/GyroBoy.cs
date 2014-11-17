using System;

namespace ev3
{
	public class GyroBoy
	{
		const float Kp = 0.5;
		const float Ki = 11;
		const float Kd = 0.005;

		const float gain_angular_velocity = 1.3; // for theta_hat
		const float gain_angle = 25;		// for theta
		const float gain_motor_speed = 75;	// for y_hat
		const float gain_motor_position = 350;	// for y

		const int max_iter = 100;
		int iter = 0;

		float refpos = 0;	// reference position
		int sample_time = 20;	// sample time in milliseconds (ms)
		float dt = (sample_time - 2)/1000;	// 
		float speed = 0;
		const int wheel_diameter = 55; // in millimeters (mm)
		int radius = wheel_diameter;
		const int max_index = 7;

		private EV3Brick ev3 = new EV3Brick();

		public GyroBoy ()
		{
		}

		public void start() {
			ev3.connect ();

//			getBalancePos();
			control();
//			shutDown();

			ev3.disconnect ();
		}

		void control() {
			while (iter++ < max_iter) {
				// Position
				refpos = refpos + (dt * speed * 0.002);

				// ReadEncoders

				// ReadGyro

				// CombineSensorValues

				// ReadConstants

				// PID

				// Errors

				//GetSteer

				// SetMotorPower

				// Wait
			}
		}

		void initialize() {
//			dt = (sample_time - 2)/1000;
		}

	}
}

