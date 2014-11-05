using System;
using System.Threading;
using MonoBrick.EV3;
using BackProp;

namespace ev3 {
	public class EV3Program{
		private const string connectionType = "/dev/tty.EV3-SerialPort";
	//	Brick<Sensor,Sensor,Sensor,Sensor> ev3;

		static void Main(string[] args)
		{
	//		testEV3 ();
	//		testBackProp ();
			testNN ();
		}

		static void testNN() {
			NeuralNetwork net = new NeuralNetwork ();
			Console.WriteLine ();
			net.learn ();
			Console.WriteLine ();

			// sensors (touch, light|color, ultrasonic) and bias 1
			// light sensor (0|1)
			double[] inputs = {0, 1};
			// motors B, C (0|1)
			double[] outputs = {1, 1};

			net.test (inputs, outputs);
		}

		static void testBackProp() {
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
				var maxIter = 20;
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

		void WaitForMotorToStop (Brick<Sensor,Sensor,Sensor,Sensor> ev3)  
		{  
			Thread.Sleep(500);  
			while(ev3.MotorC.IsRunning()){Thread.Sleep(50);}  
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