using System;
using System.Threading;
using System.Collections.Generic;

using MonoBrick.EV3;
using BackProp;
using IronPython.Hosting;

namespace ev3 {
	public class EV3Program{
		private const string connectionType = "/dev/tty.EV3-SerialPort";
		private const string pythonPath = "/System/Library/Frameworks/Python.framework/Versions/Current/lib/python2.7/";
	//	Brick<Sensor,Sensor,Sensor,Sensor> ev3;

		static void Main(string[] args)
		{
			EV3Program program = new EV3Program ();
			program.testController ();
//			program.testIronPython ();

//			program.testBackProp ();
//			program.testNN ();
//			program.testNNLineFollower ();

			EV3Brick ev3 = new EV3Brick ();
//			ev3.runH25 ();
//			ev3.testH25 ();
//			ev3.testMotorA ();
//			ev3.testMotorB ();
//			ev3.testMotorD ();

//			testEV3 ();
		}

		void testNN() {
			NeuralNetwork net = new NeuralNetwork ();
			Console.WriteLine ();
			net.learn ();
			Console.WriteLine ();

			// sensors (touch, light|color, ultrasonic) and bias 1
			// light sensor (0|1)
//			double[] inputs = {0, 0, 300, 0};
			double[] inputs = {0, 1};
			// motors B, C on/off (0|1)
//			double[] outputs = {0, 300};
			double[] outputs = {1, 0};

			net.test (inputs, outputs);
			// inputs: light 0-2, 3-8, 9-100
			// inputs: current position/status => light?
			// desired: r1 (down), r2(pick), r3(up), r4(down), r5(drop)
			// outputs: direction, speed, degree
			//			net.test (inputs, desired);

//			inputs = new double[]{0, 300, 50, 0};
//			outputs = new double[] {50, 0};
			inputs = new double[]{1, 1};
			outputs = new double[] {0, 1};

			net.test (inputs, outputs);
		}

		private void testController()
		{
			Console.WriteLine("Testing constoller ...");
			EV3Brick ev3 = new EV3Brick ();
			ev3.connect ();

			// current status: normalize
			int motorAdeg = (int) (ev3.getMotorADegree()/50.0);  
			int motorBdeg = (int) (ev3.getMotorBDegree()/300.0);

			/*
			training data = [
				[[0,0,0,1], [0,1]],		// down1
				[[0,1,1,1], [1,0]],		// pick
				[[1,1,1,0], [0,-1]],	// up1
				[[1,0,1,1], [0,1]],		// down2
				[[1,1,0,1], [-1,0]],	// drop
				[[0,1,0,0], [0,-1]]		// up2
			]
			*/
			var argv = new List<int[]> ();
			// current (A, B), desired (A, B)
			// down1
			argv.Add (new []{motorAdeg,motorBdeg,0,1});
//			argv.Add (new []{0,0,0,1}); 
			argv.Add (new []{0,1});		// targets A, B

			// up1
//			argv.Add (new []{1,1,1,0}); 
//			argv.Add (new []{motorAdeg,motorBdeg,1,0});
//			argv.Add (new []{0,-1});		// targets A, B

			var engine = Python.CreateEngine();
			var paths = engine.GetSearchPaths();
			paths.Add(pythonPath);
			engine.SetSearchPaths(paths);
			engine.GetSysModule ().SetVariable ("argv", argv);

			var scriptRuntime = Python.CreateRuntime();
			scriptRuntime.GetSysModule().SetVariable("argv", argv);

			try
			{
				dynamic result = engine.ExecuteFile("script.py");
				Console.WriteLine(result.outputs);
				var outputs = result.outputs;
				foreach (double output in outputs) {
					Console.WriteLine(output);
				}
				motorAdeg = (int) (outputs[0]*50);
				motorBdeg = (int) (outputs[1]*300);
				Console.WriteLine(motorAdeg + ", " + motorBdeg);
				ev3.moveMotorA(motorAdeg);
				ev3.moveMotorB(motorBdeg);
				Thread.Sleep(5000);
			}
			catch (Exception ex)
			{
				Console.WriteLine(
					"Oops! We couldn't execute the script because of an exception: " + ex.Message);
			}

			ev3.disconnect ();

			Console.WriteLine("Complete.");
			//			Console.ReadLine();
		}

		private void testIronPython()
		{
			Console.WriteLine("Press enter to execute the python script!");

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

			Console.WriteLine("Complete.");
//			Console.ReadLine();
		}


		void testNNLineFollower() {
			NeuralNetworkLineFollower net = new NeuralNetworkLineFollower ();
			Console.WriteLine ();
			net.learn ();
			Console.WriteLine ();

			// sensors (touch, light|color, ultrasonic) and bias 1
			// light sensor (0|1)
			double[] inputs = {0, 1};
			// motors B, C on/off (0|1)
			double[] outputs = {1, 0};

			net.test (inputs, outputs);
			// inputs: light 0-2, 3-8, 9-100
			// inputs: current position/status => light?
			// desired: r1 (down), r2(pick), r3(up), r4(down), r5(drop)
			// outputs: direction, speed, degree
			//			net.test (inputs, desired);

			inputs = new double[]{1, 1};
			outputs = new double[] {0, 1};

			net.test (inputs, outputs);
		}

		void testBackProp() {
			BackPropProgram program = new BackPropProgram ();
			program.run ();
		}

		static void testEV3()
		{
			EV3Program program = new EV3Program ();
	//		program.hello (); // motor B
	//		program.PositionControl (); // motor C
	//		program.firstProgram ();
			program.linefollow ();

			Console.WriteLine ("Complete");
		}

		// line following using PID controller
		void linefollow() {
			Console.WriteLine ("Starting line follow ...");

			var ev3 = new Brick<Sensor,Sensor,Sensor,Sensor>(connectionType);

			try{
				Console.WriteLine("Opening connection... ");
				ev3.Connection.Open();
				Console.WriteLine("Connected: " + ev3.Connection.IsConnected);

	//			Sensor lightSensor;
				float threshold = 40;
				float lasterror = 0;
				var maxIter = 10;
				var iteration = 0;
				float light;
				float error, integral = 0, derivative;
				float correction;
				float Kp = 0.1f;
				float Ki = 0; 
				float Kd = 0;
				sbyte speed = 10;

				Vehicle vehicle = ev3.Vehicle;
	//			vehicle.RightPort = MotorPort.OutB;
	//			vehicle.LeftPort = MotorPort.OutC;
	//			vehicle.ReverseLeft = false;
	//			vehicle.ReverseRight = false;

				while (iteration < maxIter) {
					light = System.Convert.ToSingle(ev3.Sensor1.ReadAsString());

					error = threshold - light;
					Console.WriteLine (error);
					integral = error + integral;
					derivative = error - lasterror;

					correction = Kp * error + Ki * integral + Kd * derivative;
					Console.WriteLine(error + " " + correction);
					// turn B+C motors by correction
	//				ev3.Vehicle.TurnRightForward(speed, (sbyte)correction);
					if(error == 0) {
	//					vehicle.Forward(speed);
					} else if (error > 0) {
						Console.WriteLine("spin left");
						ev3.MotorB.On((sbyte)correction);
						ev3.MotorC.On(speed);
	//					vehicle.SpinLeft((sbyte)correction);
	//					vehicle.TurnLeftForward(speed, (sbyte)correction);
					} else {
						Console.WriteLine("spin right");
						ev3.MotorC.On ((sbyte)correction);
						ev3.MotorB.On(speed);
	//					vehicle.SpinRight((sbyte)correction);
	//					vehicle.TurnRightForward(speed, (sbyte)correction);
					}

					lasterror = error;

					iteration ++;
					Thread.Sleep(1000);
				}
	//			vehicle.Brake();
	//			vehicle.Off();
				ev3.MotorB.Brake();
				ev3.MotorC.Brake();
				ev3.MotorB.Off();
				ev3.MotorC.Off();
	//			vehicle.LeftPort.

	//			Console.WriteLine("MotorB.on: " + ev3.MotorB.IsRunning());
	//			ev3.MotorB.On(20);
	//			Console.WriteLine("Motor B: " + ev3.MotorB.ToString());
	//			Console.WriteLine("Sleeping for 2 seconds ... ");
	//			System.Threading.Thread.Sleep(2000);
	//			Console.WriteLine("MotorB.off");
	//			ev3.MotorB.Off();

				Console.WriteLine("Program Complete");
			}
			catch(Exception e){
				Console.WriteLine(e.StackTrace);
				Console.WriteLine("Error: " + e.Message);
				Console.WriteLine("Press any key to end...");
				Console.ReadKey();				
			}
			finally{
	//			ev3.Vehicle.Off ();
	//			ev3.MotorB.Off ();
	//			ev3.MotorC.Off ();
				ev3.Connection.Close();
			}
		}

		// test Motor B
		void hello() {
			Console.WriteLine ("Hello");

			//		var ev3 = new Brick<Sensor,Sensor,Sensor,Sensor>("usb");
			var ev3 = new Brick<Sensor,Sensor,Sensor,Sensor>(connectionType);

			try{
				Console.WriteLine("Opening connection... ");
				ev3.Connection.Open();
				Console.WriteLine("Connected: " + ev3.Connection.IsConnected);
				Console.WriteLine("MotorB.on: " + ev3.MotorB.IsRunning());
				ev3.MotorB.On(20);
				Console.WriteLine("Motor B: " + ev3.MotorB.ToString());
				Console.WriteLine("Sleeping for 2 seconds ... ");
				System.Threading.Thread.Sleep(2000);
				Console.WriteLine("MotorB.off");
				ev3.MotorB.Off();
				Console.WriteLine("Program Complete");
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

		void firstProgram() {
			var brick = new Brick<Sensor,Sensor,Sensor,Sensor>(connectionType); 
			try{ 			 
				sbyte speed = 0;  
				brick.Connection.Open();  
				ConsoleKeyInfo cki;  
				ConsoleKey ck;
				Console.WriteLine("Press Q to quit");  
	//			do   
	//			{  
	//				cki = Console.ReadKey(true); //press a key  
	//				cki = new ConsoleKeyInfo(ConsoleKey.R);
	//				cki.Key = ConsoleKey.R;
				ck = ConsoleKey.UpArrow;
	//				switch(cki.Key){    
					switch(ck) {
					case ConsoleKey.R:    
						Console.WriteLine("Motor A reverse direction");    
						brick.MotorA.Reverse = !brick.MotorA.Reverse;    
						break;                              
					case ConsoleKey.UpArrow:     
						if(speed < 100)    
							speed = (sbyte)(speed + 10);    
						Console.WriteLine("Motor A speed set to " + speed);    
						brick.MotorA.On(speed);    
						break;    
					case ConsoleKey.DownArrow:     
						if(speed > -100)    
							speed = (sbyte)(speed - 10);    
						Console.WriteLine("Motor A speed set to " + speed);    
						brick.MotorA.On(speed);    
						break;    
					case ConsoleKey.S:     
						Console.WriteLine("Motor A off");    
						speed = 0;    
						brick.MotorA.Off();    
						break;    
					case ConsoleKey.B:    
						Console.WriteLine("Motor A break");    
						speed = 0;    
						brick.MotorA.Brake();    
						break;  
					case ConsoleKey.T:    
						int count = brick.MotorA.GetTachoCount();  
						Console.WriteLine("Motor A tacho count:" +count);    
						break;  
					case ConsoleKey.C:    
						Console.WriteLine("Clear tacho count");    
						brick.MotorA.ResetTacho();  
						break;  
					case ConsoleKey.M:  
						Console.WriteLine("Enter position to move to.");  
						string input = Console.ReadLine();  
						Int32 position;  
						if(Int32.TryParse(input, out position)){  
							Console.WriteLine("Move to " + position);  
							brick.MotorA.MoveTo(50, position, false);  
						}  
						else{  
							Console.WriteLine("Enter a valid number");  
						}  
						break;  
					}  
	//			} while (ck != ConsoleKey.Q); 
	//			} while (cki.Key != ConsoleKey.Q);  
			}  
			catch(Exception e){  
				Console.WriteLine("Error: " + e.Message);  
				Console.WriteLine("Press any key to end...");  
				Console.ReadKey();                
			}  
			finally{  
				brick.Connection.Close();  
			}  
		}  
	}
}