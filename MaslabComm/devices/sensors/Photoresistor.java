package devices.sensors;

import devices.actuators.DigitalOutput;

public class Photoresistor extends AnalogInput{

	private DigitalOutput led;
	private float MinValue = 3;
	private float ThreshValue = 10;
	public Photoresistor(int i) {
		super(i);
		// TODO Auto-generated constructor stub
	}

	public boolean isBall(){
		boolean output = false;
		led.setValue(true);
		if(this.value>MinValue){
			output = true;
		}
		return output;
	}
	
	public boolean isRed(){
		boolean output = false;
		if(this.value<ThreshValue){
			output =true;
		}
		return output;
	}

}
