package edu.mit.felixsun.maslab;

public class SonarReadState extends State {
	public double SCALE = 50;
	
	public SonarReadState() {
	}
	
	public void step(cvData data, Sensors sensors) {
		data.grid.set(-sensors.ultraLeft.getDistance()*SCALE, -3, 2);
		data.grid.set(sensors.ultraRight.getDistance()*SCALE, -3, 2);
	}
}
