import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import lejos.utility.Stopwatch;

public class EV3Tester {
	private static int wait = 10; // wait time in seconds

	private DifferentialPilot p;
//		Port port = LocalEV3.get().getPort("S2");
//		EV3GyroSensor gyro = new EV3GyroSensor(port);
	private EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2); // 2424ms
	private static SampleProvider angleProvider;
	private static OdometryPoseProvider opp;
	private Stopwatch watch = new Stopwatch();
	
	public static void main(String[] args) {
		EV3Tester tester = new EV3Tester();
//		tester.testGyro();
		tester.testMotors();
//		tester.testDifferentialPilot();

		tester.sleep(wait);
	}
	
	public void testMotors() {
		System.out.println("Testing Motors...");
		
		watch.reset();
		DifferentialPilot pilot = new DifferentialPilot(5.6, 9.25, Motor.A, Motor.D);
		System.out.println(watch.elapsed() + "ms"); // 1000, 1024, 1204ms
		System.out.println("DiffPilot ready."); 
		sleep(3); // in seconds
		
		watch.reset();
		pilot.setTravelSpeed(1);
		pilot.forward();
		System.out.println(watch.elapsed() + "ms"); // 81, 22, 19, 21ms
		System.out.println("forward."); 
		sleep(3); // in seconds

		watch.reset();
		pilot.setTravelSpeed(1);
		pilot.backward();
		System.out.println(watch.elapsed() + "ms"); // 23, 85, 70, 83ms
		System.out.println("backward."); 
		sleep(3); // in seconds
		
		watch.reset();
		pilot.quickStop();
		System.out.println(watch.elapsed() + "ms"); // 127, 136, 160, 107ms
		System.out.println("stopped."); 
		sleep(3); // in seconds
		
		// SegowayPilotDemo demo;
//		fail("Not yet implemented");
	}

	public void testGyro() {
		System.out.println("Testing GyroSensor...");
				
//		EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2); // 2424ms
		
		watch.reset();
		angleProvider = gyro.getAngleMode();
		System.out.println(watch.elapsed() + "ms"); // 20, 6ms, 6ms, 20ms,1ms
		System.out.println("angleProvider ready."); 
		sleep(3); // in seconds
		
		watch.reset();
		gyro.reset();
		System.out.println(watch.elapsed() + "ms"); // 21, 10ms, 9ms, 12ms, 9ms
		System.out.println("GyroSensor reset."); 
		sleep(3); // in seconds
		
		watch.reset();
		float[] sample = new float[angleProvider.sampleSize()]; // sample size 1
		angleProvider.fetchSample(sample, 0);
		System.out.println(watch.elapsed() + "ms");  // 30, 32ms, 32ms, 31ms, 4ms
		System.out.println("sample fetched.");
		sleep(3); // in seconds
		
		watch.reset();
		System.out.println("sample size = " + angleProvider.sampleSize());
		System.out.println(watch.elapsed() + "ms");  // 26, 21ms, 13ms, 19ms, 4ms
		System.out.println("sample fetched.");
		sleep(3); // 2 seconds
		
		System.out.println("sample[0]: " + sample[0]);
	}
	
	public void testDifferentialPilot() {
		System.out.println("Testing DiffPilot...");
		
		p = new DifferentialPilot(5.6, 9.25, Motor.A, Motor.D);
//		lejos.hardware.port.Port port = LocalEV3.get().getPort("S2");
//		EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2); // 2424ms
//		gyro = new EV3GyroSensor(port2);
		p.setTravelSpeed(30);
		p.setRotateSpeed(360);
		int accel = 60;
		p.setAcceleration(accel);
		opp = new OdometryPoseProvider(p);
		gyro.reset();
		Sound.beepSequenceUp();
		p.forward();
//		report("START acceleration " + accel);
		/*p.rotate(45);
		report("R 45");
		p.rotate(90);
		report("R 90");
		p.rotate(45);
		report("R 45");
		p.rotate(-180);
		report("R -180");*/
	}

	public void report(String message) {
		Delay.msDelay(100);
		float heading = opp.getPose().getHeading();
		float[] angle = { 0 };
		angleProvider.fetchSample(angle, 0);
		System.out.println(message + " H " + heading + " A " + angle[0]);
	}

	private void sleep(int seconds) {
		try {
			Thread.sleep(seconds * 1000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
