using System;
using System.Threading;
using System.Diagnostics;
//using System.Timers;

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
		const int drive_sleep = 7000; // milliseconds

		//bool sound = false;
        bool sound = true;

        float refpos = 0;	// reference position
		const int sample_time = 22;	// sample time in milliseconds (ms)
		const float dt = (sample_time - 2)/1000f;	// verified
		float speed = 0;
		const int wheel_diameter = 55; // in millimeters (mm)
		const float radius = wheel_diameter / 2000f; // verified

		const int max_index = 7;
		float[] enc_val = new float[max_index];
		int enc_index = 0;

		bool nowOutOfBound = false;
		bool prevOutOfBound = false;
		int outOfBoundCount = 0;
//		int outOfBound = 0;

		float ang = 0;
//		float mean_ang = 0;
		float mean = 0;
		int steering = 0;
		float old_steering = 0;
		int max_acceleration = 0;

		const float radius_const = 57.3f;
		float acc_err = 0;
		float prev_err = 0;

		const string FORMAT = "0.00"; // precision

		bool complete = false;

		Stopwatch stopwatch;

		private EV3Brick ev3 = new EV3Brick();

		public GyroBoy ()
		{
		}

		// verified
		public void start() {
			ev3.connect ();

			// verified
			initialize ();

			// verified
			Thread t1 = new Thread (new ThreadStart (balance));
			t1.Start ();
			// verified
			Thread t2 = new Thread (new ThreadStart (drive));
			t2.Start ();

			t1.Join ();
			t2.Join ();

			ev3.disconnect ();
		}

		// verified
		// controls speed and steering
		void drive() {
			Console.WriteLine ("driving ...");

//			while (true) {
			while (!complete) {
				speed = 0;
				Thread.Sleep (drive_sleep);
				speed = -20;
				Thread.Sleep (drive_sleep);
				speed = 20;
				Thread.Sleep (drive_sleep);
			}
		}

		// verified
		void balance() {
			Console.WriteLine("refpos = " + refpos + ", dt = " + dt + ", Kp = " + Kp + ", Ki = " + Ki + ", Kd = " + Kd);
			Console.WriteLine ("iter\tspeed\tang_vel\tang"
				+ "\tsensor\tavg_pwr\toffset\trefpos"
				+ "\tmspeed"
				+ "\tspeedA\tspeedD\textra\tpwr_b\tpwr_c"
//				+ "\tcurr_err\tacc_err\tdif_err\tprev_err"
			);

            Stopwatch totalwatch = Stopwatch.StartNew();
            Stopwatch functionwatch = Stopwatch.StartNew();
            int iter = 0;

            stopwatch.Restart();

            while (iter++ < max_iter) {
				// Position: verified
				refpos = refpos + (dt * speed * 0.002f);

                // ReadEncoders: verified
                //functionwatch.Restart();
				float motor_speed = (radius * getMotorSpeed ()) / radius_const;
				float motor_position = (radius * (ev3.getMotorADegree () + ev3.getMotorDDegree ()) / 2.0f) / radius_const;
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
                //Console.WriteLine(functionwatch.ElapsedMilliseconds + "ms");

                // Wait: verified
                // Timer >= dt, elapsedTime
                long elapsedTime = stopwatch.ElapsedMilliseconds;
                Console.WriteLine(elapsedTime + " " + totalwatch.ElapsedMilliseconds);
				if(elapsedTime >= dt * 1000f)
                    stopwatch.Restart();
            }

			complete = true;
		}

		// verified
		void initialize() {
			Console.WriteLine ("initializing ...");
//			dt = (sample_time - 2)/1000;

			// erase array
			for (int i = 0; i < max_index; i++) {
				enc_val [i] = 0;
			}

            stopwatch = Stopwatch.StartNew();
            ev3.resetMotorATachoCount ();
			ev3.resetMotorDTachoCount();
            Console.WriteLine("Reset tacho count: " + (stopwatch.ElapsedMilliseconds/2f).ToString(FORMAT) + "ms");

            Thread.Sleep (100);

            // verified
            stopwatch.Restart();
            mean = calibrate ();
            Console.WriteLine(stopwatch.ElapsedMilliseconds + "ms");

			speed = 0;
			steering = 0;

            // reset timer 
            stopwatch.Reset();
		}

		/**
		 * verified
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

			int count = 20;

			// gyro rate
			for (int i = 0; i < count; i++) {
				mean += gyroRate ();
				//Console.WriteLine ("gyroRate mean: " + mean);
				Thread.Sleep (5);
			}
			mean = mean / count;

            //Console.WriteLine ("gyroRate mean: " + mean);

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
//			Console.WriteLine (enc_val [enc_index] + " " + enc_val[compare_index] + " " + max_index + " " + dt);

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
			const float ref_val = 0;

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
			//Console.WriteLine (speedA.ToString(FORMAT) + "\t" + speedD.ToString(FORMAT) 
			//	+ "\t" + extra_pwr.ToString(FORMAT) + "\t" + pwr_b.ToString(FORMAT) + "\t" + pwr_c.ToString(FORMAT));
		}

		// verified except the interrupting balance loop
		public void errors(float avg_pwr) {
			nowOutOfBound = (Math.Abs (avg_pwr) > 100f);

			// read cur_err

			if (nowOutOfBound && prevOutOfBound) {
				outOfBoundCount++;
			} else {
				outOfBoundCount = 0;
			}

			if (outOfBoundCount > 20) {
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

