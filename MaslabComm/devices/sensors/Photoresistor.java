package devices.sensors;

import devices.actuators.DigitalOutput;

public class Photoresistor extends AnalogInput{

	private DigitalOutput led;
	private float MaxValue = 4000;
	private float ThreshValue = 10;
	public Photoresistor(int i, int j) {
		super(i);
		led = new DigitalOutput(j);
	}

	public boolean isBall(){
		boolean output = false;
		led.setValue(true);
		System.out.println(this.value);
		if(this.value<MaxValue){
			output = true;
		}
		led.setValue(false);
		return output;
	}
	
	public boolean isRed(){
		boolean output = false;
		led.setValue(true);
		if(this.value<ThreshValue){
			output =true;
		}
		led.setValue(false);
		return output;
	}

}
