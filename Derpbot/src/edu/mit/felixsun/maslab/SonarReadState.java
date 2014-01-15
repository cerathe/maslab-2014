package edu.mit.felixsun.maslab;

public class SonarReadState extends State {
	public double SCALE = 45;
	public double HALF_WIDTH = 5;
	
	public SonarReadState() {
	}
	
	public void step(cvData data, Sensors sensors) {
		for (double y = -2; y < 3; y += data.gridSize) {
		data.grid.set(-sensors.ultraLeft.getDistance()*SCALE - HALF_WIDTH, y, 2);
		data.grid.set(sensors.ultraRight.getDistance()*SCALE + HALF_WIDTH, y, 2);
		}
	}
}
