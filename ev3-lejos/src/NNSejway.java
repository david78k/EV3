import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.EncoderMotor;
import lejos.utility.Delay;

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

public class NNSejway 
{
    // PID constants from others
//    final float KP = 1.556f;
//    final float KI = 0.222f;
//    final float KD = 1.833f;

//  final float KP = 1.7f;
//  final float KI = 0.222f;
//  final float KD = 2.3f;

	// (1.5, 0.01, 20) working more stable with 1 sample
	// 5 samples
	// (10, 0.1, 23) working very stable
	// (10, 0.15, 23) working too much oscillation
	// (10, 0.2, 23) working robust to disturbance, best 
	// (13, 0.222, 23) working easily falls down 
	// (8, 0.222, 23) working unstable 
    final static float KP = 10f; // 1.5f working, 5 better, 1 bit slow, 3/10 good, 15/20 too fast, default 28
    final static float KI = 0.2f; // 0.5 large oscillation, 0.01 working, 0.00001/0.01 better, 0.001/0.1 good, 1 too fast, default 4, depends on sample time dt
    final static float KD = 23f; // 0/10 working, 0.001/0.01/0.1 good, 1 too fast, default 33
    // PID constants
//	kp = 0.0336f;
//	ki = 0.2688f;
//	kd = 0.000504f;
//	private static final float Kp = 0.5f;  // default 0.5f
//	private static final float Ki = 11;   // default 11
//	private static final float Kd = 0.005f; // default 0.005f
//    final int SCALE = 1;  // default 18
    final static int base_power = 20; // in percentage, 30 bit fast, 10 not moving, default 20 good
    final static int max_power = 100; // %, default 100% (max), shouldn't be over 100
    
    EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2);
	EncoderMotor leftMotor = new NXTMotor(MotorPort.A); 
	EncoderMotor rightMotor = new NXTMotor(MotorPort.D); 
	
	// Global vars:
	float offset = 0;
	float prev_error;
	float int_error;
	
	// assume that network is already trained
	// NeuralNetwork nn = new NeuralNetwork();
	// NeuralNetwork nn = new NeuralNetwork(weights);
	
	public static void main(String[] args) 
	{
		NNSejway sej = new NNSejway();
		sej.start();
	}
	
    public void start() {
    	// nn.train();
    	gyro.reset();
    	for(int i = 0; i < 20; i ++) {
    		offset += gyroRate();
    	}
    	offset /= 20f;
    	
//		sej.getBalancePos();
//    	pidControl();
    	control();
    	shutDown();
    	
    	Delay.msDelay(10000);
    }
    
    float gyroRate() {
    	return gyroRate(1);
    }
    
    /**
	 * average of samples of angular velocity
	 */
    float gyroRate(int sample_size) {
		float filter = 0;

		// get samples
		int offset = 0;
		float[] sample = new float[sample_size];
		gyro.getRateMode().fetchSample(sample, offset);
		for(int i = 0; i < sample_size; i ++)
			filter += sample[i];
		
		return filter / sample_size;
	}
    
    public void control() {
    	System.out.println("control");
        while (!Button.ESCAPE.isDown()) 
        {
        	float normVal = gyroRate(5);  // [-440, 440]
//        	float normVal = gyroRate();

            // Proportional Error:
            float error = normVal - offset;
            // Adjust far and near light readings:
//            if (error < 0) error = (int)(error * 1.8F);

//            int u = (int) nnControl(error);
            int u = (int) pidControl(error);
			
            // may need to change to check if outbound count > 20
            if (u > max_power)
                u = max_power;
            if (u < max_power)
                u = max_power;

            // Power derived from PID value:
            int power = Math.abs(u);
//            power = 55 + (power * 45) / 100; // Default NORMALIZE POWER 55 + => [55,100%]
            power = base_power + (power * (100 - base_power)) / 100; // [10,100%]
//            System.out.println(normVal + " " + pid_val + " " + power);
            
            int sign = -1 * (int) Math.signum(u);
            leftMotor.setPower(sign*power);
            rightMotor.setPower(sign*power);
        }
        System.out.println("complete");
    }
	
    // neural network control
    // assume that network is already trained and weights are known
    public float nnControl(float error) {
    	// return nn.test(error);
    	return 0;
    }
    
    public float pidControl(float error) 
    {
//    	System.out.println("PID control");

    	// Proportional Error: error

    	// Integral Error:
    	int_error = (int_error + error);

    	// Derivative Error:
    	float deriv_error = error - prev_error;
    	prev_error = error;

    	return (KP * error + KI * int_error + KD * deriv_error);
    }
	
    // Shut down light sensor, motors
    public void shutDown()
    {
    	leftMotor.flt();
    	rightMotor.flt();
    }
    
    public void getBalancePos() 
    {
    	// Wait for user to balance and press orange button
    	while (!Button.ENTER.isDown())
    	{
    		// NXTway must be balanced.
//    		offset = ls.readNormalizedValue();
    		LCD.clear();
//    		LCD.drawInt(offset, 2, 4);
    		LCD.refresh();
    	}
    }
}
