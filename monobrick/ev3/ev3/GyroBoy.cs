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
			}
		}

		void initialize() {
		}

	}
}

