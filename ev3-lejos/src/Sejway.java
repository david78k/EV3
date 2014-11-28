import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.EncoderMotor;
import lejos.robotics.navigation.DifferentialPilot;

//import lejos.nxt.*;

/**
 * A controller for a self-balancing Lego robot with a light sensor
 * on port 2. The two motors should be connected to port B and C.
 *
 * Building instructions in Brian Bagnall: Maximum Lego NXTBuilding 
 * Robots with Java Brains</a>, Chapter 11, 243 - 284
 * 
 * @author Brian Bagnall
 * @version 26-2-13 by Ole Caprani for leJOS version 0.9.1
 */

public class Sejway 
{
    // PID constants
    final int KP = 28;
    final int KI = 4;
    final int KD = 33;
    final int SCALE = 18;

//    LightSensor ls;
    EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2);
    private DifferentialPilot pilot = new DifferentialPilot(5.6, 9.25, Motor.A, Motor.D);
	EncoderMotor leftMotor = new NXTMotor(MotorPort.A); 
	EncoderMotor rightMotor = new NXTMotor(MotorPort.D); 
	
	// Global vars:
	int offset;
	int prev_error;
	float int_error;
	
	public static void main(String[] args) 
	{
		Sejway sej = new Sejway();
//		sej.getBalancePos();
		sej.pidControl();
		sej.shutDown();
	}
	
    public Sejway() 
    {
//        ls = new LightSensor(SensorPort.S2, true);
    	gyro.reset();
    }
	
    public void getBalancePos() 
    {
    	// Wait for user to balance and press orange button
    	while (!Button.ENTER.isDown())
    	{
    		// NXTway must be balanced.
//    		offset = ls.readNormalizedValue();
    		LCD.clear();
    		LCD.drawInt(offset, 2, 4);
    		LCD.refresh();
    	}
    }
	
    /**
	 * verified
	 * average of 5 samples of angular velocity
	 */
	float gyroRate() {
		float filter = 0;

		// get 5 samples
		float[] sample = new float[5];
		int offset = 0;
		gyro.getRateMode().fetchSample(sample, offset );
//		gyro.getAngleMode().fetchSample(sample, offset );
		for(int i = 0; i < 5; i ++)
			filter += sample[i];
//			filter = ev3.getAngularVelocity () + filter;
		
		return filter / 5f;
	}
	
    public void pidControl() 
    {
        while (!Button.ESCAPE.isDown()) 
        {
//            int normVal = ls.readNormalizedValue();
        	int normVal = (int) gyroRate();

            // Proportional Error:
            int error = normVal - offset;
            // Adjust far and near light readings:
            if (error < 0) error = (int)(error * 1.8F);
			
            // Integral Error:
            int_error = ((int_error + error) * 2)/3;
			
            // Derivative Error:
            int deriv_error = error - prev_error;
            prev_error = error;
			
            int pid_val = (int)(KP * error + KI * int_error + KD * deriv_error) / SCALE;
			
            if (pid_val > 100)
                pid_val = 100;
            if (pid_val < -100)
                pid_val = -100;

            // Power derived from PID value:
            int power = Math.abs(pid_val);
            power = 55 + (power * 45) / 100; // NORMALIZE POWER

            leftMotor.setPower(power);
            rightMotor.setPower(power);
            if (pid_val > 0) {
//                MotorPort.B.controlMotor(power, BasicMotorPort.FORWARD);
//                MotorPort.C.controlMotor(power, BasicMotorPort.FORWARD);
            	leftMotor.forward();
            	rightMotor.forward();
            } else {
//                MotorPort.B.controlMotor(power, BasicMotorPort.BACKWARD);
//                MotorPort.C.controlMotor(power, BasicMotorPort.BACKWARD);
            	leftMotor.forward();
            	rightMotor.forward();
            }
        }
    }
	
    public void shutDown()
    {
        // Shut down light sensor, motors
//        Motor.B.flt();
//        Motor.C.flt();
//        ls.setFloodlight(false);
    	leftMotor.flt();
    	rightMotor.flt();
    }
}