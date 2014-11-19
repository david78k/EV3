using System;
using System.Threading;
using System.Collections.Generic;
using MonoBrick.EV3;

namespace ev3
{
	public class EV3Brick
	{
		private const string connectionType = "/dev/tty.EV3-SerialPort";
		private Brick<Sensor,Sensor,Sensor,Sensor> ev3 = new Brick<Sensor,Sensor,Sensor,Sensor>(connectionType);

		public EV3Brick ()
		{
		}

		public void connect() {
			try {
				Console.WriteLine ("Opening connection... ");
				ev3.Connection.Open ();
				Console.WriteLine ("Connected: " + ev3.Connection.IsConnected);

//				motor.ResetTacho ();  
//				Console.WriteLine ("Position: " + motor.GetTachoCount ());
			} catch (Exception e) {
				Console.WriteLine (e.StackTrace);
				Console.WriteLine ("Error: " + e.Message);
				Console.WriteLine ("Press any key to end...");
				Console.ReadKey ();				
			} finally {
//				ev3.Connection.Close ();
//				Console.WriteLine ("Connection closed.");
			}
		}

		public void disconnect() {
			try {
				Console.WriteLine ("Closing connection... ");
				ev3.Connection.Close ();
				Console.WriteLine ("Connected: " + ev3.Connection.IsConnected);
			} catch (Exception e) {
				Console.WriteLine (e.StackTrace);
				Console.WriteLine ("Error: " + e.Message);
				Console.WriteLine ("Press any key to end...");
				Console.ReadKey ();				
			} finally {
//				stopAll ();
				ev3.Connection.Close ();
				Console.WriteLine ("Connection closed.");
			}
		}

		public void sound(int volume, int frequency, int durationMs) {
			ev3.PlayTone ((byte)volume, (ushort)frequency, (ushort)durationMs);
		}

		public void onMotorA(int speed) {
			ev3.MotorA.On ((sbyte)speed);
		}

		public void onMotorB(int speed) {
			ev3.MotorB.On ((sbyte)speed);
		}

		public void onMotorC(int speed) {
			ev3.MotorC.On ((sbyte)speed);
		}

		public void onMotorD(int speed) {
			ev3.MotorD.On ((sbyte)speed);
		}

		public void offMotorA() {
			ev3.MotorA.Off ();
		}

		public void offMotorB() {
			ev3.MotorB.Off ();
		}

		public void offMotorC() {
			ev3.MotorC.Off ();
		}

		public void offMotorD() {
			ev3.MotorD.Off ();
		}

		public void setPowerMotorA(int power) {
			ev3.MotorA.SetPower ((byte)power);
		}

		public void setPowerMotorB(int power) {
			ev3.MotorB.SetPower ((byte)power);
		}

		public void setPowerMotorC(int power) {
			ev3.MotorC.SetPower ((byte)power);
		}

		public void setPowerMotorD(int power) {
			ev3.MotorD.SetPower ((byte)power);
		}

		/**
		 * Assuming Sensor2 to be GyroSensor
		 */
		public int getAngularVelocity() {
			string value = ev3.Sensor2.ReadAsString ();
//			Console.WriteLine ("Gyro: " + value);
			return Convert.ToInt16(value);
		}

		public void getMotorDegrees() {

		}

		public int getMotorADegree() {
			return ev3.MotorA.GetTachoCount ();
		}

		public int getMotorBDegree() {
			return ev3.MotorB.GetTachoCount ();
		}

		public int getMotorCDegree() {
			return ev3.MotorC.GetTachoCount ();
		}

		public int getMotorDDegree() {
			return ev3.MotorD.GetTachoCount ();
		}

		public void resetMotorBTachoCount() {
			ev3.MotorB.ResetTacho ();
		}

		public void resetMotorATachoCount() {
			ev3.MotorA.ResetTacho ();
		}

		public void resetMotorCTachoCount() {
			ev3.MotorC.ResetTacho ();
		}

		public void resetMotorDTachoCount() {
			ev3.MotorD.ResetTacho ();
		}

		public void stopAll() {
			ev3.MotorA.Brake ();
			ev3.MotorA.Off ();
			ev3.MotorD.Brake ();
			ev3.MotorD.Off ();
		}

		// direction = -1 or 1
		public void moveMotorA(int degree) {
			int direction = (degree > 0 ? 1 : -1);
			Console.WriteLine ("Moving motor A " + degree + "(" + (uint)Math.Abs(degree) + ") ...");
			ev3.MotorA.On ((sbyte)(direction * 5), (uint)Math.Abs(degree), true);
			WaitForMotorToStop (ev3.MotorA);
			Console.WriteLine ("Motor A stopped.");
		}

		public void moveMotorB(int degree) {
			Console.WriteLine ("is connected? " + ev3.Connection.IsConnected);
			int direction = (degree > 0 ? 1 : -1);
			Console.WriteLine ("Moving motor B " + degree + " (" + (uint)Math.Abs(degree) + ") ..." + (sbyte)(direction*5));
			ev3.MotorB.On ((sbyte)(direction * 5), (uint)Math.Abs(degree), true);
			WaitForMotorToStop (ev3.MotorB);
			Console.WriteLine ("Motor B stopped.");
		}

		public void testMotorB() {
			Console.WriteLine ("Testing motor B ...");
			Motor motor = ev3.MotorB;

			try{
				Console.WriteLine("Opening connection... ");
				ev3.Connection.Open();
				Console.WriteLine("Connected: " + ev3.Connection.IsConnected);

				motor.ResetTacho();  
//				motor.On(5, (uint)(50),true);  
				motor.On(5, (uint)(310),true);  // down +
				// pick up the ball
//				motor.On(-5, (uint)(300),true);  // up -
				WaitForMotorToStop(motor);  
				//				ev3.MotorB.Off();
				Console.WriteLine("Position: " + motor.GetTachoCount());

				Console.WriteLine("Program Complete.");
			}
			catch(Exception e){
				Console.WriteLine(e.StackTrace);
				Console.WriteLine("Error: " + e.Message);
				Console.WriteLine("Press any key to end...");
				Console.ReadKey();				
			}
			finally{
				motor.Off ();
				ev3.Connection.Close();
				Console.WriteLine ("Connection closed.");
			}
		}

		public void testMotorA() {
			Console.WriteLine ("Testing motor A ...");
			Motor motor = ev3.MotorA;

			try{
				Console.WriteLine("Opening connection... ");
				ev3.Connection.Open();
				Console.WriteLine("Connected: " + ev3.Connection.IsConnected);

				motor.ResetTacho();  
				// drop or pick up the ball
				//				ev3.MotorA.On(5, (uint)(70),true);  // pick +
				motor.On(-5, (uint)(100),true);  // drop -
				WaitForMotorToStop(motor);  
				motor.Off();

				Console.WriteLine("Program Complete.");
			}
			catch(Exception e){
				Console.WriteLine(e.StackTrace);
				Console.WriteLine("Error: " + e.Message);
				Console.WriteLine("Press any key to end...");
				Console.ReadKey();				
			}
			finally{
				motor.Off ();
				ev3.Connection.Close();
				Console.WriteLine ("Connection closed.");
			}
		}

		public void testMotorD() {
			Console.WriteLine ("Testing motor D ...");
			Motor motor = ev3.MotorD;

			try{
				Console.WriteLine("Opening connection... ");
				ev3.Connection.Open();
				Console.WriteLine("Connected: " + ev3.Connection.IsConnected);

				motor.ResetTacho();  

				// drop or pick up the ball
				//				ev3.MotorD.On(5, (uint)(100),true);  
				motor.On(-5, (uint)(80),true);  
				WaitForMotorToStop(motor);  
				ev3.MotorD.Off();

				Console.WriteLine("Program Complete.");
			}
			catch(Exception e){
				Console.WriteLine(e.StackTrace);
				Console.WriteLine("Error: " + e.Message);
				Console.WriteLine("Press any key to end...");
				Console.ReadKey();				
			}
			finally{
				motor.Off ();
				ev3.Connection.Close();
				Console.WriteLine ("Connection closed.");
			}
		}
		/*
		void runH25() {
			var argv = new List<int[]> ();
			//			args.ToList().ForEach(a => argv.Add(a));
			argv.Add (new []{0,1,1,1});
			argv.Add (new []{1,0});

			var engine = Python.CreateEngine();
			var paths = engine.GetSearchPaths();
			paths.Add(pythonPath);
			engine.SetSearchPaths(paths);
			engine.GetSysModule ().SetVariable ("argv", argv);

			var scriptRuntime = Python.CreateRuntime();
			scriptRuntime.GetSysModule().SetVariable("argv", argv);
			//			scriptRuntime.ExecuteFile("script.py");

			try
			{
				//				engine.ExecuteFile("bpnn.py");
				dynamic result = engine.ExecuteFile("script.py");
				Console.WriteLine(result.outputs);
				var outputs = result.outputs;
				foreach (double output in outputs) {
					Console.WriteLine(output);
				}
			}
			catch (Exception ex)
			{
				Console.WriteLine(
					"Oops! We couldn't execute the script because of an exception: " + ex.Message);
			}
		}
*/
		void testH25() {
			Console.WriteLine ("Starting H25 ...");

//			var ev3 = new Brick<Sensor,Sensor,Sensor,Sensor>(connectionType);

			try{
				Console.WriteLine("Opening connection... ");
				ev3.Connection.Open();
				Console.WriteLine("Connected: " + ev3.Connection.IsConnected);

				//				ev3.MotorA.On(10);
				//				Console.WriteLine("Sleeping for 2 seconds ... ");
				//				System.Threading.Thread.Sleep(2000);
				//				ev3.MotorA.Off();

				ev3.MotorB.ResetTacho();  
				//				ev3.MotorB.On(-10, (uint)(0.5*360),true);  
				//				ev3.MotorB.On(-5, (uint)(0.5*360),true);  
				//				ev3.MotorB.On(-10, (uint)(0.1*360),true);  
				//				ev3.MotorB.On(-5, (uint)(0.1*360),true);  
				//				ev3.MotorB.On(10, (uint)(0.5*360),true);  
				//				ev3.MotorB.On(10, (uint)(0.3*360),true);  
				//				ev3.MotorB.On(5, (uint)(0.1*360),true);  
				//				ev3.MotorB.On(5, (uint)(0.5*360),true);  
				//				ev3.MotorB.On(3, (uint)(0.5*360),true);  
				Sensor lightSensor = getLightSensor(ev3);
				string status = "GROUND";
				measure(ev3);
				int light = measureLight(lightSensor);

				// pick up the ball
				//				ev3.MotorA.On(10, (uint)(100),true);  
				ev3.MotorA.On(-10, (uint)(100),true);  

				// move up
				while(light < 8) {
					ev3.MotorB.On(-10, (uint)(0.5*360),true);  
					WaitForMotorToStop(ev3);  
					light = measureLight(lightSensor);
				}				
				Console.WriteLine();
				WaitForMotorToStop(ev3); 
				ev3.MotorB.Off();
				measure(ev3);
				Console.WriteLine("Position: TOP (" + ev3.MotorB.GetTachoCount() + ")");
				Console.WriteLine("Moving down ...");

				// move down to the ground
				while(light > 1) {
					ev3.MotorB.On(5, (uint)(0.5*360),true);  
					WaitForMotorToStop(ev3);  
					light = measureLight(lightSensor);
				}				
				Console.WriteLine();
				WaitForMotorToStop(ev3);  
				ev3.MotorB.Off();
				measure(ev3);
				Console.WriteLine("Position: GROUND (" + ev3.MotorB.GetTachoCount() + ")");  

				// drop the ball
				ev3.MotorA.On(10, (uint)(100),true);  
				//				ev3.MotorA.On(-10, (uint)(100),true);  
				WaitForMotorToStop(ev3);  
				ev3.MotorA.Off();

				Console.WriteLine("Program Complete.");
			}
			catch(Exception e){
				Console.WriteLine(e.StackTrace);
				Console.WriteLine("Error: " + e.Message);
				Console.WriteLine("Press any key to end...");
				Console.ReadKey();				
			}
			finally{
				ev3.Connection.Close();
				Console.WriteLine ("Connection closed.");
			}
		}

		public void measure(Brick<Sensor,Sensor,Sensor,Sensor> ev3) {
			SensorType[] stypes = ev3.GetSensorTypes();
			//
			//			foreach (SensorType stype in stypes) {
			//				Console.WriteLine (stype);
			//			}
			measure (ev3.Sensor1); 
			measure (ev3.Sensor2);
			measure (ev3.Sensor3);
			measure (ev3.Sensor4);
			Console.WriteLine ();
		}

		public void measure(Sensor sensor) {
			SensorType stype = sensor.GetSensorType();
			// Touch: 0 TOUCH                         
			// Color: 53 COL-REFLECT                   
			// UltraSonic: 0 US-DIST-CM 
			string sename = sensor.GetName ();
			string sname = "";

			string[] snames = { "TOUCH", 
				"COL-REFLECT", "COL-AMBIENT", "COL-COLOR", 
				"US-DIST-CM", "US-DIST-IN", "US-LISTEN" };

			foreach (string s in snames) {
				if (sename.Contains (s))
					sname = s;
			}

			switch (stype) {
			case SensorType.Color: 
			case SensorType.Touch:
			case SensorType.UltraSonic:
				Console.Write (stype + " (" + sname + "): ");
				Console.Write (sensor.ReadAsString () + "    ");
				//				Console.WriteLine (sname);
				break;
			default:
				break;
			}
			//			if(stype != SensorType.None)
			//				Console.WriteLine ();
		}

		// Color: 53 COL-REFLECT                   
		public int measureLight(Sensor sensor) {
			int light = -1;

			SensorType stype = sensor.GetSensorType();
			string sename = sensor.GetName ();
			string sname = "";

			string[] snames = { "TOUCH", 
				"COL-REFLECT", "COL-AMBIENT", "COL-COLOR", 
				"US-DIST-CM", "US-DIST-IN", "US-LISTEN" };

			foreach (string s in snames) {
				if (sename.Contains (s))
					sname = s;
			}

			switch (stype) {
			case SensorType.Color: 
				Console.Write (stype + " (" + sname + "): ");
				Console.WriteLine (sensor.ReadAsString () + "    ");
				light = Convert.ToInt16(sensor.ReadAsString ());
				break;
			default:
				break;
			}			

			return light;
		}

		public Sensor getLightSensor(Brick<Sensor,Sensor,Sensor,Sensor> ev3) {
			if (ev3.Sensor1.GetSensorType () == SensorType.Color)
				return ev3.Sensor1;
			else if (ev3.Sensor2.GetSensorType () == SensorType.Color)
				return ev3.Sensor2;
			else if (ev3.Sensor3.GetSensorType () == SensorType.Color)
				return ev3.Sensor3;
			else if (ev3.Sensor4.GetSensorType () == SensorType.Color)
				return ev3.Sensor4;
			else
				return null;
		}


		// test Motor C
		void PositionControl()  
		{  
			var ev3 = new Brick<Sensor,Sensor,Sensor,Sensor>(connectionType);

			try{  
				ev3.Connection.Open();  
				ev3.MotorC.ResetTacho();  
				ev3.MotorC.On(50, 6*360,true);  
				WaitForMotorToStop(ev3);  
				Console.WriteLine("Position: " + ev3.MotorC.GetTachoCount());  
				ev3.MotorC.On(-50, 9*360, true);  
				WaitForMotorToStop (ev3);  
				Console.WriteLine("Position: " + ev3.MotorC.GetTachoCount());  
				ev3.MotorC.MoveTo(50,0,true);  
				WaitForMotorToStop(ev3);  
				ev3.MotorC.Off();  
				Console.WriteLine("Position: " + ev3.MotorC.GetTachoCount());  
			}  
			catch(Exception e){  
				Console.WriteLine(e.StackTrace);  
				Console.WriteLine("Error: " + e.Message);  
				Console.WriteLine("Press any key to end...");  
				Console.ReadKey();                
			}  
			finally{  
				ev3.Connection.Close();  
			}  
		}  

		void WaitForMotorToStop (Motor motor)  
		{  
			Thread.Sleep(500);  
			while(motor.IsRunning()){Thread.Sleep(50);}  
		}  

		void WaitForMotorToStop (Brick<Sensor,Sensor,Sensor,Sensor> ev3)  
		{  
			Thread.Sleep(500);  
			while(ev3.MotorA.IsRunning() || ev3.MotorB.IsRunning()
				|| ev3.MotorC.IsRunning() 
				|| ev3.MotorD.IsRunning())
			{Thread.Sleep(50);}  
		}  
	}
}

