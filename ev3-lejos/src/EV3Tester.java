import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.EncoderMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;
import lejos.utility.Stopwatch;
import lejos.utility.Timer;

public class EV3Tester {

	private EncoderMotor leftMotor = new NXTMotor(MotorPort.A);
	private EncoderMotor rightMotor = new NXTMotor(MotorPort.D);
	private EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2); // 2424ms
	private static final int TIMEOUT = 30*1000; // escape timeout in milliseconds
	private static int wait = 10; // wait time in seconds
	
	private static SampleProvider angleProvider;
	private Stopwatch watch = new Stopwatch();
	
	private static OdometryPoseProvider opp;
	private DifferentialPilot p;
	
	public static void main(String[] args) {
		EV3Tester tester = new EV3Tester();
		tester.testGyro();
//		tester.testMotors();
//		tester.testDifferentialPilot();

//		tester.sleep(wait);
		tester.escape();
	}
	
	public void testGyroAndMotors() {
		System.out.println("Testing Gyro and Motors...");
		
	}
	
	public void testGyro() {
		System.out.println("Testing Gyro...");
				
		int size = 5;
		float[] sample = new float[size]; 
		
//		watch.reset();
		gyro.reset();
		gyro.getRateMode().fetchSample(sample, 0);
		System.out.println("sample size = " + size);
		
		for (float f : sample) {
			System.out.printf("sample: %.2f\n", f);
		}
	}
	
	public void testMotors() {
		System.out.println("Testing Motors...");
		resetTachoCounts();
		printTachoCounts();
		
		int power = 30; // %
		setPower(power);
		Delay.msDelay(2);
		printTachoCounts();
		
		power = -30;
		Delay.msDelay(2);
		setPower(power);
		
		power = 30;
		setPower(power);
		Delay.msDelay(2);
		printTachoCounts();
		
		power = 30;
//		lejos.utility.Timer timer = new Timer(theDelay, el);
		Delay.msDelay(2000);
		// ideally the tacho count has to 450 (750 * 0.3 = 225, 225 * 2 seconds = 450)
		// count in the air = 429 445
		// count on the table = 340 354 (due to friction)
		printTachoCounts();
		
//		escape();
		floatMotors();
	}
	
	void printTachoCounts() {
		System.out.println(leftMotor.getTachoCount() + " " + rightMotor.getTachoCount());
	}
	
	void resetTachoCounts() {
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
	}
	void setPower(int power) {
		leftMotor.setPower(power);
		rightMotor.setPower(power);
	}
	
	void floatMotors() {
		leftMotor.flt();
		rightMotor.flt();
	}
	
	void escape() {
//		while(!Button.ESCAPE.isDown())
//			Delay.msDelay(1);
		Button.waitForAnyEvent(TIMEOUT);
	}
	
	public void testPilot() {
		System.out.println("Testing Pilot...");
		
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

	public void testGyroSampleTime() {
		System.out.println("Testing GyroSampleTime...");
				
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
