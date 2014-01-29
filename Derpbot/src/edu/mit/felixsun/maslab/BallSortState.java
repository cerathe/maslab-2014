package edu.mit.felixsun.maslab;



import devices.actuators.DigitalOutput;
import devices.actuators.Servo;
import devices.sensors.AnalogInput;

public class BallSortState extends State {

	public AnalogInput res;
	public DigitalOutput led;
	public Servo servo;
	public boolean sorted = false;
	public int countdown;
	double midAngle;
	double maxAngle;
	double minAngle;
	BallDetectState detect;
	boolean toSort = false;
	public BallSortState(Sensors sensors){
		res = sensors.photoresistor;
		led = sensors.led;
		servo = sensors.sorter;
		countdown = 0;
		BallDetectState detect = new BallDetectState(sensors); 
		midAngle = (servo.getMaxAngle() + servo.getMinAngle())/2;
		maxAngle = servo.getMaxAngle()-5;
		minAngle = servo.getMinAngle()+5;

	}

	public int step(double ballThresh, double greenThresh){
		/* toGo = 1: ball has been detected, try to sort;
		 * toGo = 0: no ball; return 0;
		 */ 
		
		if(countdown==0){
			//If we aren't sorting, check if we need to.
			if(!toSort){
				System.out.println("No ball :(");
				servo.setAngle(midAngle);
				int toGo = detect.step(ballThresh);
				if(toGo==1){
					toSort = true;
				}
				return 0;
			}
			toSort = false;
			
			//If we are sorting, sort.
			boolean isRed = false;
			System.out.println(res.getValue());
			if(res.getValue()<greenThresh){
				//empirical observation that green does not return false reds.
				isRed = true;
			}
			System.out.println(isRed);
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
			led.setValue(false);
		}
		else{
			countdown--;
			//			System.out.println(countdown);
		}
		return 1;
	}
}
