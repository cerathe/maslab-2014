package edu.mit.felixsun.maslab;

import devices.actuators.DigitalOutput;
import devices.sensors.AnalogInput;

public class BallSortState extends State {
	
	public AnalogInput res;
	public DigitalOutput led;
	
	public BallSortState(Sensors sensors){
		res = sensors.photoresistor;
		led = sensors.led;
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
			if(res.getValue()>ballThresh){
				existsBall = true;
				if(res.getValue()>greenThresh){
					System.out.println("A Green Ball!");
				}
				else{
					//empirical observation that green does not return false reds.
					System.out.println("A Red Ball!");
					isRed = true;
				}
			}
		}
		if(existsBall){ return 0;}
		else{
			//TODO: Move the servos
			if(isRed){
				System.out.println("RED!");
			}
			else{
				System.out.println("GREEN!");
			}
			return 1;
		}
		
	}
}
