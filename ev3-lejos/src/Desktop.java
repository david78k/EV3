//import tool.Tool4Robot;
//import tool.Tool4Throwable;
import lejos.hardware.Button;
//import lejos.hardware.ButtonListener;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.HiTechnicGyro;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.utility.Delay;

/**
 * The <CODE>DeskTop</CODE> class controls a vehicle, which remains on the desktop.
 * <P><B>title: DeskTop</B></P>
 * <P>description: Use of the motors:<br>
 * right motor: port B.<br>
 * left motor: port A.<br>
 * Use of the sensors:<br>
 * IR sensor: port S2.<br>
 * touch sensor: port S1.<br>
 * gyro sensor: port S3.</P>
 * <P>copyright: (c) 2014 thomas kaffka, born at 11/08/1959 in düsseldorf - germany , all rights reserved.</P>
 * @author thomas kaffka , cologne , germany.
 * @version 21.01.2014 18:02:46
 * @since 1.0
 */

/*
public class DeskTop implements ButtonListener {

   private DifferentialPilot robot;

   private SampleProvider distance;
   private SampleProvider touch;
   private SampleProvider gyro;

   private Tool4Robot t4r;

   *//**
    * The constructor <CODE>DeskTop</CODE> creates a new instance(object) of the class.
    * @since 1.0
    *//*
   public DeskTop() {

      try {

         t4r = new Tool4Robot();
         t4r.beepMessageOk();
   
         LCD.clear();
         LCD.drawString(t4r.LCD_center("DeskTop"), 0, 0);
         LCD.drawString(t4r.LCD_center("-------"), 0, 1);
         t4r.drawBattery(2);
   
         // initialize the motors (0.5.0-alpha).
         LCD.drawString("initialize motors", 0, 3);
         double diam = DifferentialPilot.WHEEL_SIZE_NXT2;
         double trackwidth = 10.5;
         robot = new DifferentialPilot(diam, trackwidth, Motor.A, Motor.B);
         robot.setTravelSpeed(7);
         robot.setRotateSpeed(30);
         Delay.msDelay(4000);
   
         // initialize the sensors.
         LCD.drawString("initialize sensors", 0, 3);
         Port port = LocalEV3.get().getPort("S2");
         EV3IRSensor sensor1 = new EV3IRSensor(port);
         distance = sensor1.getDistanceMode();
         
         port = LocalEV3.get().getPort("S1");
         EV3TouchSensor sensor2 = new EV3TouchSensor(port);
         touch = sensor2.getTouchMode();

         port = LocalEV3.get().getPort("S3"); // 1.4 to -8.5 is quiet.
         HiTechnicGyro sensor3 = new HiTechnicGyro(port);
         gyro = sensor3.getMode(0);
         Delay.msDelay(1000);
   
         // initialize the buttons.
         Button.ESCAPE.addButtonListener(this);
   
         LCD.drawString("ready...          ", 0, 3);
         t4r.beepMessageReady();
         Delay.msDelay(1000);

      } catch (Exception ex) {
         t4r.beepMessageError();
         System.out.println(Tool4Throwable.getCallStackOut(ex));
      }
   }

   *//**
    * The method <CODE>avoid</CODE> avoids an obstacle.
    * @since 1.0
    *//*
   public void avoid() {
      robot.stop();
      Button.LEDPattern(Tool4Robot.RED_ON); 
      t4r.beepBeepTwo();
      Delay.msDelay(1000);
        
      int random = (int) (Math.random() * 10.0) + 7;
      Button.LEDPattern(Tool4Robot.ORANGE_DOUBLE); 
      robot.travel(-random);
        
      random = (int) (Math.random() * 100.0);
      int factor = 1;
      if (random < 50) {
         factor = -1;
      }
      random = (int) (Math.random() * 80.0) + 10;
      random *= factor;
      robot.rotate(random);
      Delay.msDelay(500);
      
      Button.LEDPattern(Tool4Robot.GREEN_BLINK); 
      robot.forward();
   }
   
   *//**
    * The method <CODE>go</CODE> lets the robot move.
    * @since 1.0
    *//*
   public void go() {
      
      LCD.drawString("go...             ", 0, 3);
      Delay.msDelay(1000);

      float[] distance_sample = new float[distance.sampleSize()];
      float[] touch_sample = new float[touch.sampleSize()];
      float[] gyro_sample = new float[gyro.sampleSize()];

      Button.LEDPattern(Tool4Robot.GREEN_BLINK); 
      robot.forward();
      
      try {
         
         while(Button.ESCAPE.isUp()) {
            
            // fetch the sensor data.
            distance.fetchSample(distance_sample, 0);
            touch.fetchSample(touch_sample, 0);
            gyro.fetchSample(gyro_sample, 0);

            // avoid if obstacle in front.
            if (distance_sample[0] > 10) {
               avoid();
            } else if (touch_sample[0] == 1.0) {
               avoid();
            } else if (gyro_sample[0] > 1.4 || gyro_sample[0] < -19.5) {
               avoid();
            }

            Delay.msDelay(50);
         }

          Button.LEDPattern(Tool4Robot.PATTERN_OFF);
          t4r.beepMessageEnd();

        } catch (Exception ex) {
         t4r.beepMessageError();
         System.out.println(Tool4Throwable.getCallStackOut(ex));
      }
   }

   *//**
    * The method <CODE>main</CODE> starts the program.
    * @param args the command line arguments.
    * @since 1.0
    *//*
   public static void main(String[] args) {
      DeskTop dt = new DeskTop();
      dt.go();
   }

   @Override
   public void buttonPressed(Button b) {
      t4r.beepDown();
      Button.LEDPattern(Tool4Robot.PATTERN_OFF);
      System.exit(0);
   }

   @Override
   public void buttonReleased(Button b) {
      // TODO Auto-generated method stub
      
   }

}
*/