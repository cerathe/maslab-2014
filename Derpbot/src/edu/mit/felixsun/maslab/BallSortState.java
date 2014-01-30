package edu.mit.felixsun.maslab;



import devices.actuators.DigitalOutput;
import devices.actuators.Servo;
import devices.sensors.AnalogInput;

public class BallSortState extends State {

	int ballDelay = 10;
	int sortDelay = 10;
	int pushDelay = 30;

	public AnalogInput res;
	public DigitalOutput redled;
	public DigitalOutput greenled;
	public Servo servo;

	//switches
	boolean isBall = false;
	boolean isSorted = false;
	boolean isRed = false;

	//counters
	int ballCounter = ballDelay;
	int colorCounter = sortDelay;
	int sortCounter = pushDelay;

	//leds
	double redVal = -1;
	double greenVal = -1;
	boolean redOn = false;
	boolean greenOn = false;

	double midAngle;
	double maxAngle;
	double minAngle;
	BallDetectState detect;
	boolean toSort = false;
	public BallSortState(Sensors sensors){
		res = sensors.photoresistor;
		redled = sensors.redled;
		greenled = sensors.greenled;
		servo = sensors.sorter;

		midAngle = (servo.getMaxAngle() + servo.getMinAngle())/2+10;
		maxAngle = servo.getMaxAngle()-5;
		minAngle = servo.getMinAngle()+5;
		sensors.sorter.setAngle(midAngle);
	}

	public int step(double ballThresh, double redThresh, double greenThresh){
		/*
		 * 2: ball has been successfully sorted; ok to try again.
		 * 1: ball has been detected, try to sort;
		 * 0: no ball; return 0;
		 */ 

		//check if there is a ball by making sure you get valid values for n straight ticks
		if(!isBall){
			if(ballCounter==0){
				isBall=true;
				isSorted = false;
				colorCounter = sortDelay;
				greenled.setValue(false);
				greenOn = false;
				colorCounter = sortDelay;
				return 1;
			}
			else if(res.getValue()>ballThresh){
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
			if(redVal==-1 && redOn && !greenOn){
				//keep red on for some ticks;
				if(colorCounter==0){
					redVal = -res.getValue();
					redled.setValue(false);
					redOn = false;
					greenled.setValue(true);
					greenOn = true;
					colorCounter = sortDelay;
					return 3;
				}
				else{
					colorCounter--;
					return 1;
				}
			}
			else if(greenVal==-1 && greenOn && !redOn){
				//keep green on for some ticks;
				if(colorCounter==0){
					greenVal = res.getValue();
					greenled.setValue(false);
					greenOn = false;
					return 3;
				}
				else{
					colorCounter--;
					return 1;
				}
			}
			else{
				isSorted = true;
				sortCounter = pushDelay;
				return 1;
			}
		}
		//sort it.

		if(sortCounter == pushDelay){
			sortCounter--;
			if(greenVal>greenThresh && redVal<redThresh){
				servo.setAngle(minAngle);
				return 1;
			}
			else if(greenVal<greenThresh && redVal> redThresh){
				servo.setAngle(maxAngle);
				return 5;
			}
			else{
				isBall = false;
				isSorted = false;
				ballCounter  = ballDelay;
				redVal = -1;
				greenVal = -1;
				redOn = true;
				greenOn =true;
				redled.setValue(true);
				greenled.setValue(true);
				return 0;
			}
		}
		else if(sortCounter>0){
			sortCounter--;
			return 4;
		}
		else{
			servo.setAngle(midAngle);
			redled.setValue(true);
			greenled.setValue(true);
			redVal = -1;
			greenVal = -1;
			redOn = true;
			greenOn = true;
			ballCounter = ballDelay;
			isBall = false;
			isSorted = false;
			return 2;
		}
	}
}
