package edu.mit.felixsun.maslab;

public class SonarReadState extends State {
	
	int state;
	
	public SonarReadState() {
		state = 0;
	}
	
	public void step(cvData data) {
		System.out.println(data.ultra1.getDistance());
	}
}
