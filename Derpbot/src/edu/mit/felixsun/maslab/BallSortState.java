package edu.mit.felixsun.maslab;



import com.sun.org.apache.bcel.internal.generic.CompoundInstruction;

import comm.CommInterface;
import comm.MapleComm;
import devices.actuators.DigitalOutput;
import devices.actuators.Servo;
import devices.sensors.AnalogInput;

public class BallSortState extends State {
	
	public AnalogInput res;
	public DigitalOutput led;
	public Servo servo;
	public boolean sorted = false;
	public int countdown;
	
	public BallSortState(Sensors sensors){
		res = sensors.photoresistor;
		led = sensors.led;
		servo = sensors.sorter;
		countdown = 0;
	}
	
	public int step(double ballThresh, double greenThresh){
		//Green returns no false positives for red.
		//Red returns false positives for green.
		//If we see Red, it's definitely red.
		//If we see green we should remeasure 5 times.
		led.setValue(true);
		
		double midAngle = (servo.getMaxAngle() + servo.getMinAngle())/2;
		double maxAngle = servo.getMaxAngle()-5;
		double minAngle = servo.getMinAngle()+5;
		
		boolean existsBall = false;
		boolean isRed = false;
			System.out.println(res.getValue());
			if(res.getValue()>ballThresh){
				existsBall = true;
				if(res.getValue()<greenThresh){
					//empirical observation that green does not return false reds.
					isRed = true;
				}
			}
		System.out.println(existsBall + " " + isRed);
//		led.setValue(false);
		if(!existsBall){
			System.out.println("No ball :(");
			servo.setAngle(midAngle);
			return 0;
		}
		else if(countdown==0){
			//TODO: Move the servos
			countdown = 20;
			if(isRed){
				System.out.println("RED!");
				//sort to one side
				servo.setAngle(minAngle);
				sorted = true;
			}
			else{
				System.out.println("GREEN!");
				//sort to other side
				servo.setAngle(maxAngle);
				sorted = true;
			}
		}
		else{
			countdown--;
//			System.out.println(countdown);
		}
		return 1;
	}
}
