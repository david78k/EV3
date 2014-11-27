import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;

public class EV3Tester {
	
	private DifferentialPilot p;
	private EV3GyroSensor gyro;
	private static SampleProvider angleProvider;
	private static OdometryPoseProvider opp;

	public static void main(String[] args) {
		EV3Tester tester = new EV3Tester();
//		tester.testGyro();
//		tester.testMotors();
		tester.testSensors();
	}
	
	public void testMotors() {
		System.out.println("Testing Motors...");
//		fail("Not yet implemented");
//		Motor leftmotor = 
		DifferentialPilot pilot;
		// SegowayPilotDemo demo;
	}

	public void testGyro() {
		System.out.println("Testing GyroSensor...");
//		Port port = LocalEV3.get().getPort("S2");
//		EV3GyroSensor gyro = new EV3GyroSensor(port);
		EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2);
		angleProvider = gyro.getAngleMode();
	}
	
	public void testSensors() {
		System.out.println("Testing DiffPilot...");
		
		p = new DifferentialPilot(5.6, 9.25, Motor.A, Motor.D);
//		lejos.hardware.port.Port port = LocalEV3.get().getPort("S2");
		Port port2 = SensorPort.S2;
		gyro = new EV3GyroSensor(port2);
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

}
