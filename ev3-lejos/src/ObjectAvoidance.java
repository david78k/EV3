import java.io.File;

import neuralnet.NeuralNetwork;
import lejos.hardware.Button;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.EncoderMotor;
import lejos.utility.Delay;

public class ObjectAvoidance {

//	private static final boolean DEBUG = true;
	private static final boolean DEBUG = false;

	private static final int max_iter = 10;
			
	EV3UltrasonicSensor ultra = new EV3UltrasonicSensor(SensorPort.S4); // ultrasonic
	EncoderMotor leftMotor = new NXTMotor(MotorPort.A); 
	EncoderMotor rightMotor = new NXTMotor(MotorPort.D); 
	File logfile = new File("/home/root/avoidance.log");
	
	public static void main(String[] args) {
		ObjectAvoidance oa = new ObjectAvoidance();
		oa.runNeuralNetwork();
//		oa.run();
	}

	public void runNeuralNetwork() {
		
		int speed = 0;
		float dist = 0;
		float[] sample = new float[1];
		
		int i = 0;
		while(i++ < max_iter && !Button.ESCAPE.isDown()) {
			speed = 50;
			// move straight
			leftMotor.setPower(speed);
			rightMotor.setPower(speed);
			while(true) {
				// motor rotation degree > 25
				// in meter
				ultra.getDistanceMode().fetchSample(sample, 0);
				dist = sample[0];
				Delay.msDelay(200);
				if(dist <= 0.3) break;
			}
			// object detected
			System.out.println(dist);
			
			// move back
			speed = -30;
			leftMotor.setPower(speed);
			rightMotor.setPower(speed);
			Delay.msDelay(2000);
			
			// move right
			// steering = 30;
			speed = -50;
			leftMotor.setPower(speed);
			speed = 50;
			rightMotor.setPower(speed);
			Delay.msDelay(2000);
		}
		Delay.msDelay(10000);
	}
	
	public void run() {
		int speed = 0;
		float dist = 0;
		float[] sample = new float[1];
		
		int i = 0;
		while(i++ < max_iter && !Button.ESCAPE.isDown()) {
			speed = 50;
			// move straight
			leftMotor.setPower(speed);
			rightMotor.setPower(speed);
			while(true) {
				// motor rotation degree > 25
				// in meter
				ultra.getDistanceMode().fetchSample(sample, 0);
				dist = sample[0];
				Delay.msDelay(200);
				if(dist <= 0.3) break;
			}
			// object detected
			System.out.println(dist);
			// stop
//			speed = 0;
//			leftMotor.setPower(speed);
//			rightMotor.setPower(speed);
//			Delay.msDelay(2000);
			
			// move back
			speed = -30;
			leftMotor.setPower(speed);
			rightMotor.setPower(speed);
			Delay.msDelay(2000);
			
			// move right
			// steering = 30;
			speed = -50;
			leftMotor.setPower(speed);
			speed = 50;
			rightMotor.setPower(speed);
			Delay.msDelay(2000);
		}
		Delay.msDelay(10000);
	}
}
