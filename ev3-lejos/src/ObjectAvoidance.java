import java.io.File;

import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.EncoderMotor;
import lejos.utility.Delay;

public class ObjectAvoidance {

//	private static final boolean DEBUG = true;
	private static final boolean DEBUG = false;

	EV3UltrasonicSensor ultra = new EV3UltrasonicSensor(SensorPort.S4); // ultrasonic
	EncoderMotor leftMotor = new NXTMotor(MotorPort.A); 
	EncoderMotor rightMotor = new NXTMotor(MotorPort.D); 
	File logfile = new File("/home/root/avoidance.log");
	
	public static void main(String[] args) {

	}

	public void run() {
		int speed = 0;
		float dist = 0;
		float[] sample = new float[1];
		
		while(true) {
			speed = 50;
			while(true) {
				// motor rotation degree > 25
				ultra.getDistanceMode().fetchSample(sample, 0);
				dist = sample[0];
				Delay.msDelay(200);
				if(dist <= 30) break;
			}
		}
	}
}
