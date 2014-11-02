using System;
using MonoBrick.EV3;

public static class Program{
	static void Main(string[] args)
	{
		Console.WriteLine ("Hello");

//		var ev3 = new Brick<Sensor,Sensor,Sensor,Sensor>("usb");
		var ev3 = new Brick<Sensor,Sensor,Sensor,Sensor>("/dev/tty.EV3-SerialPort");

		try{
			Console.WriteLine("Opening connection... ");
			ev3.Connection.Open();
			Console.WriteLine("Connected: " + ev3.Connection.IsConnected);
			Console.WriteLine("MotorB.on: " + ev3.MotorB.IsRunning());
			ev3.MotorB.On(5);
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
}