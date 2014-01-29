package edu.mit.felixsun.maslab;

import devices.actuators.DigitalOutput;
import devices.actuators.Servo;
import devices.sensors.AnalogInput;

public class BallDetectState {
	public DigitalOutput led;
	public AnalogInput res;
	public boolean sorted = false;
	public int countdown;
	
	public BallDetectState(Sensors sensors){
		led = sensors.led;
		res = sensors.photoresistor;
		countdown = 0;
	}
	
	public int step(double ballThresh){
		System.out.println(res.getValue());
		if(res.getValue()<ballThresh){
			//empirical observation that green does not return false reds.
			led.setValue(true);
			return 1;
		}
		return 0;	
	}
}
