import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.Audio;
import lejos.hardware.Bluetooth;
import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.Sound;
import lejos.hardware.port.IOPort;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.AnalogSensor;
import lejos.hardware.sensor.BaseSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3SensorConstants;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.hardware.sensor.EV3SensorConstants;
import lejos.remote.ev3.RemoteEV3;

/*
import josx.platform.rcx.TextLCD;
import josx.platform.rcx.Sound;
import josx.platform.rcx.Sensor;
import josx.platform.rcx.SensorConstants;
import josx.platform.rcx.Motor;
import josx.platform.rcx.Button;
*/

/*
 * Two touch sensors, one light sensor, one infrared tower
 * Two motors
 */
public class LMbpnDemo {
    public static LMbpn bpn = new LMbpn();

    public static void main(String args[]) throws InterruptedException
    {
        int i, white;
        int inp[] = {0,0,0};
        int out[] = {0,0};

//        RemoteEV3 ev3 = RemoteEV3.
        EV3 ev3 = (EV3) BrickFinder.getDefault();
        Keys keys = ev3.getKeys();

        Sound.beep();
        
        //        TextLCD.print( "Train" );
        TextLCD lcd = ev3.getTextLCD();
        lcd.drawString("Train", 4, 4);
        // Train bpn 500 epochs, sit down and wait about 5 minutes!
        for(i=0;i<500;i++) {
        	bpn.train(1);
            lcd.drawInt( bpn.trainedEpochs, 4, 4 );
            //LCD.showNumber( bpn.trainedEpochs );
        }
/*
        Sensor.S1.setTypeAndMode ( SensorConstants.SENSOR_TYPE_TOUCH,
                                   SensorConstants.SENSOR_MODE_BOOL );
        Sensor.S2.setTypeAndMode ( SensorConstants.SENSOR_TYPE_LIGHT,
                                   SensorConstants.SENSOR_MODE_RAW );
        Sensor.S3.setTypeAndMode ( SensorConstants.SENSOR_TYPE_TOUCH,
                                   SensorConstants.SENSOR_MODE_BOOL );
*/
        EV3TouchSensor tsensor = new EV3TouchSensor(new Port() {
			
			@Override
			public <T extends IOPort> T open(Class<T> arg0) {
				// TODO Auto-generated method stub
				return null;
			}
			
			@Override
			public int getSensorType() {
				// TODO Auto-generated method stub
				return 0;
			}
			
			@Override
			public int getPortType() {
				// TODO Auto-generated method stub
				return 0;
			}
			
			@Override
			public String getName() {
				// TODO Auto-generated method stub
				return null;
			}
		});
        
        Sound.twoBeeps();
        /*
        Sound.twoBeeps();
        Sensor.S2.activate();   
        white = Sensor.S2.readRawValue();
        Motor.A.setPower(1);
        Motor.C.setPower(1);
        Sound.twoBeeps();
        while( !Button.PRGM.isPressed() ) {
            LCD.showNumber( Sensor.S2.readRawValue() );
            if( Sensor.S1.readBooleanValue() )
                inp[0] = 1; // Sensor 1 on
            else
                inp[0] = 0; // Sensor 1 off
            if( Sensor.S2.readRawValue() > white + 50 )
                inp[1] = 1; // Sensor 2 over black floor
            else
                inp[1] = 0; // Sensor 2 over white floor
            if( Sensor.S3.readBooleanValue() )
                inp[2] = 1; // Sensor 3 on
            else
                inp[2] = 0; // Sensor 3 off
            bpn.test( inp, out );
            if( out[0] == 1 )
                Motor.A.forward();
            else
                Motor.A.backward();
            if( out[1] == 1 )
                Motor.C.forward();
            else
                Motor.C.backward();
            Thread.sleep( 500 );
        } // while()
        Sensor.S2.passivate();
        Motor.A.stop();
        Motor.C.stop();
        Sound.beep();
*/
    } // main()
} // class LMbpn
