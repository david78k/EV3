using System;
using System.Threading;

namespace ev3
{
	/**
	 * for NXT
	 */
	public class Sejway
	{
		// PID constants
		const int KP = 28; // originally 28
		const int KI = 4;	// originally 4
		const int KD = 33;	// origianlly 33
		const int SCALE_PID = 18;  // originally 18
		const int SCALE_POWER =100; // orginally 100
		const int SPEED = 55; // originally 55
		const int SLEEP = 10; // in milliseconds

		// Global vars:
		int offset = 0;
		int prev_error = 0;
		float int_error = 0;

		int iter = 0;
		const int max_iter = 100;

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
//				if (error < 0) error = (int)(error * 1.8F);

				// Integral Error:
				int_error = ((int_error + error) * 2)/3;

				// Derivative Error:
				int deriv_error = error - prev_error;
				prev_error = error;

				int pid_val = (int)(KP * error + KI * int_error + KD * deriv_error) / SCALE_PID;

				if (pid_val > 100)
					pid_val = 100;
				if (pid_val < -100)
					pid_val = -100;

				// Power derived from PID value:
				int power = Math.Abs(pid_val);
				power = SPEED + (power * 45) / SCALE_POWER; // NORMALIZE POWER

				Console.Write (iter + "\t" + normVal + "\t" + error + "\t"
				+ pid_val + "\t" + power + "\t");

				if (pid_val >= 0) {
					Console.WriteLine ("Forward");
					ev3.onMotorA (power);
					ev3.onMotorD (power);
//					MotorPort.B.controlMotor(power, BasicMotorPort.FORWARD);
//					MotorPort.C.controlMotor(power, BasicMotorPort.FORWARD);
				} else {
					Console.WriteLine ("Backward");
					ev3.onMotorA (-1*power);
					ev3.onMotorD (-1*power);
//					MotorPort.B.controlMotor(power, BasicMotorPort.BACKWARD);
//					MotorPort.C.controlMotor(power, BasicMotorPort.BACKWARD);
				}

//				Thread.Sleep (SLEEP);
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

