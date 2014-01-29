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

	public BallSortState(Sensors sensors){
		res = sensors.photoresistor;
		led = sensors.led;
		servo = sensors.sorter;
		countdown = 0;
		midAngle = (servo.getMaxAngle() + servo.getMinAngle())/2;
		maxAngle = servo.getMaxAngle()-5;
		minAngle = servo.getMinAngle()+5;

	}

	public int step(int shouldIGo, double greenThresh){
		/* shouldIGo = 1: ball has been detected, try to sort;
		 * shouldIGo = 0: no ball; return 0;
		 */
		if(shouldIGo==0 && countdown==0){
			System.out.println("No ball :(");
			servo.setAngle(midAngle);
			return 0;
		}
		
		boolean isRed = false;
		System.out.println(res.getValue());
		if(res.getValue()<greenThresh){
			//empirical observation that green does not return false reds.
			isRed = true;
		}
		System.out.println(isRed);
		if(countdown==0){
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
