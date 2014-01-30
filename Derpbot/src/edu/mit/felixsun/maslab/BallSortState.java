package edu.mit.felixsun.maslab;



import devices.actuators.DigitalOutput;
import devices.actuators.Servo;
import devices.sensors.AnalogInput;

public class BallSortState extends State {

	int ballDelay = 10;
	int sortDelay = 5;
	int pushDelay = 10;

	public AnalogInput res;
	public DigitalOutput led;
	public Servo servo;
	
	//switches
	boolean isBall = false;
	boolean isSorted = false;
	boolean isRed = false;
	
	//counters
	int ballCounter = ballDelay;
	int colorCounter = sortDelay;
	int sortCounter = pushDelay;
	
	double midAngle;
	double maxAngle;
	double minAngle;
	BallDetectState detect;
	boolean toSort = false;
	public BallSortState(Sensors sensors){
		res = sensors.photoresistor;
		led = sensors.led;
		servo = sensors.sorter;

		detect = new BallDetectState(sensors); 
		midAngle = (servo.getMaxAngle() + servo.getMinAngle())/2;
		maxAngle = servo.getMaxAngle()-5;
		minAngle = servo.getMinAngle()+5;

	}

	public int step(double redThresh, double greenThresh){
		/*
		 *  2: ball has been successfully sorted; ok to try again.
		 * 1: ball has been detected, try to sort;
		 * 0: no ball; return 0;
		 */ 

		//check if there is a ball by making sure you get valid values for n straight ticks
		if(!isBall){
			if(ballCounter==0){
				isBall=true;
				colorCounter = sortDelay;
				return 1;
			}
			else if(res.getValue()>redThresh){
				ballCounter--;
				return 0;
			}
			else{
				ballCounter = ballDelay;
				return 0;
			}
		}
		//if there is a ball, try to sort it.
		if(!isSorted){
			if(colorCounter==0){
				isSorted = true;
				if(res.getValue()>greenThresh){
					servo.setAngle(minAngle);
					sortCounter = pushDelay;
					return 1;
				}
				else if(res.getValue()>redThresh) {
					servo.setAngle(maxAngle);
					sortCounter = pushDelay;
					return 1;
				}
				else{
					isBall = false;
					ballCounter = ballDelay;
					return 0;
				}
			}
			else{
				colorCounter --;
				return 1;
			}
		}
		//if you've sorted it, wait before replacing the sorter.

		if(sortCounter==0){
			servo.setAngle(midAngle);
			ballCounter = ballDelay;
			isBall = false;
			isSorted = false;
			isRed = false;
			return 2;
		}
		else{
			sortCounter--;
			return 1;
		}
	}
}
