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

		public GyroBoy ()
		{
		}

		void initialize() {
		}

		void setConstants() {

		}
	}
}

