using System;

namespace ev3
{
	public class Sejway
	{
		// PID constants
		const int KP = 28;
		const int KI = 4;
		const int KD = 33;
		const int SCALE = 18;

		// Global vars:
		int offset;
		int prev_error;
		float int_error;

		int iter = 0;
		const int max_iter = 10;

//		LightSensor ls;
		EV3Brick ev3;

		public Sejway() 
		{
//			ls = new LightSensor(SensorPort.S2, true);
			ev3 = new EV3Brick ();
		}

//		public static void main(String[] args) 
		public void start() 
		{
			ev3.connect ();

//			Sejway sej = new Sejway();
			getBalancePos();
			pidControl();
			shutDown();

			ev3.disconnect ();
		}

		public void getBalancePos() 
		{
			// Wait for user to balance and press orange button
//			while (!Button.ENTER.isDown())
//			{
				// NXTway must be balanced.
//				offset = ls.readNormalizedValue();
				offset = ev3.getAngularVelocity ();
//				LCD.clear();
//				LCD.drawInt(offset, 2, 4);
//				LCD.refresh();
//			}
		}

		public void pidControl() 
		{
			Console.WriteLine ("iter\tnormVal\terror\tpid_val\tpower");

//			while (!Button.ESCAPE.isDown()) 
//			while (true) 
			while (iter++ < max_iter) 
			{
//				int normVal = ls.readNormalizedValue();
				int normVal = ev3.getAngularVelocity ();

				// Proportional Error:
				int error = normVal - offset;
				// Adjust far and near light readings:
				if (error < 0) error = (int)(error * 1.8F);

				// Integral Error:
				int_error = ((int_error + error) * 2)/3;

				// Derivative Error:
				int deriv_error = error - prev_error;
				prev_error = error;

				int pid_val = (int)(KP * error + KI * int_error + KD * deriv_error) / SCALE;

				if (pid_val > 100)
					pid_val = 100;
				if (pid_val < -100)
					pid_val = -100;

				// Power derived from PID value:
				int power = Math.Abs(pid_val);
				power = 55 + (power * 45) / 100; // NORMALIZE POWER

				Console.WriteLine (iter + "\t" + normVal + "\t" + error + "\t"
				+ pid_val + "\t" + power);

				if (pid_val > 0) {
					Console.WriteLine ("Forward");
//					MotorPort.B.controlMotor(power, BasicMotorPort.FORWARD);
//					MotorPort.C.controlMotor(power, BasicMotorPort.FORWARD);
				} else {
					Console.WriteLine ("Backward");
//					MotorPort.B.controlMotor(power, BasicMotorPort.BACKWARD);
//					MotorPort.C.controlMotor(power, BasicMotorPort.BACKWARD);
				}
			}
		}

		public void shutDown()
		{
			// Shut down light sensor, motors
			ev3.stopAll ();
//			Motor.B.flt();
//			Motor.C.flt();
//			ls.setFloodlight(false);
		}
	}
}

