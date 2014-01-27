package edu.mit.felixsun.maslab;

import devices.actuators.DigitalOutput;
import devices.actuators.Servo;
import devices.sensors.AnalogInput;

public class BallSortState extends State {
	
	public AnalogInput res;
	public DigitalOutput led;
	public Servo servo;
	
	public BallSortState(Sensors sensors){
		res = sensors.photoresistor;
		led = sensors.led;
		servo = sensors.sorter;
	}
	
	public int step(double ballThresh, double greenThresh){
		//Green returns no false positives for red.
		//Red returns false positives for green.
		//If we see Red, it's definitely red.
		//If we see green we should remeasure 5 times.
		led.setValue(true);
		//wait 1 sec  to let the photoresistor come to equilibrium.
		try {
		    Thread.sleep(1000);
		} catch(InterruptedException ex) {
		    Thread.currentThread().interrupt();
		}
		boolean existsBall = false;
		boolean isRed = false;
		for(int i=0; i<5; i++){
		//measure 5 times. Green returns green all 5 times, red has some error.
		//but should return at least once.
			//jiggle the ball--this tends to improve reading accuracy.
			servo.setAngle(80);
			servo.setAngle(84);
			if(res.getValue()>ballThresh){
				existsBall = true;
				if(res.getValue()<greenThresh){
					//empirical observation that green does not return false reds.
					isRed = true;
				}
			}
		}
		servo.setAngle(82.5);
		led.setValue(false);
		if(!existsBall){
			System.out.println("No ball :(");
			return 0;
		}
		else{
			//TODO: Move the servos
			if(isRed){
				System.out.println("RED!");
				//sort to one side
				servo.setAngle(5);
				servo.setAngle(82.5);
			}
			else{
				System.out.println("GREEN!");
				//sort to other side
				servo.setAngle(160);
				servo.setAngle(82.5);
			}
			return 1;
		}
		
	}
}
