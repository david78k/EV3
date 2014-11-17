
import java.rmi.RemoteException;

import lejos.hardware.BrickFinder;
import lejos.hardware.BrickInfo;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.remote.ev3.RMIRegulatedMotor;
import lejos.remote.ev3.RemoteEV3;
import lejos.utility.Delay;

public class RemoteControl {
   
   RemoteEV3 ev3 = null;
   RMIRegulatedMotor motorA = null;
   RMIRegulatedMotor motorB = null;
   RMIRegulatedMotor motorC = null;
   RMIRegulatedMotor motorD = null;
   
   //FrontDorOpen
   private boolean isOpen = false;   
   public boolean getIsOpen() { return isOpen; }
   
   public static void main(String[] args) {
	   RemoteControl rc = new RemoteControl();
   }
   
   public RemoteControl(){
      setup();
   }
   
   private void setup() {
      try {
    	  EV3 ev = (EV3) BrickFinder.getDefault();
         BrickInfo[] bricks = BrickFinder.discover();
         for(BrickInfo info: bricks) {
            System.out.println("Ev3 found on Bluetooth ip: " + info.getIPAddress());
         }
         
         ev3 = new RemoteEV3(bricks[0].getIPAddress());
//         ev3 = (RemoteEV3) BrickFinder.getDefault();
//         EV3 ev = (EV3) BrickFinder.getDefault();
         
      } catch (Exception e) {
         e.printStackTrace();
      }
      
      ev3.setDefault();
      GraphicsLCD display = ev3.getGraphicsLCD();
      display.clear();
      Sound.beep();
      
      motorA = ev3.createRegulatedMotor("A",'l');
      motorB = ev3.createRegulatedMotor("B",'l');
      motorC = ev3.createRegulatedMotor("C",'l');
      motorD = ev3.createRegulatedMotor("D",'l');
   
      Sound.beep();
      try {         
    	  // test the first motor
         motorA.rotate(360);
      } catch (RemoteException e) {
         e.printStackTrace();         
      }      
   }
   public boolean forward(int sek) throws RemoteException{
      motorA.forward();
      motorB.forward();
      Delay.msDelay(sek*1000);
      motorA.stop(false);
      motorB.stop(false);
      return false;
   }

}