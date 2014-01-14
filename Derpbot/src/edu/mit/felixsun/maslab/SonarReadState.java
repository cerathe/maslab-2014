package edu.mit.felixsun.maslab;

public class SonarReadState extends State {
	
	int state;
	
	public SonarReadState() {
		state = 0;
	}
	
	public byte[] step(cvData data) {
		if (state == 0) {
			System.out.println(data.serialOut);
			byte[] output = {'I', 'U', 1, 23, 24, 127};
			state = 1;
			return output;
		} else if (state == 1) {
			
			byte[] output = {'G'};
			return output;
		} else {
			return new byte[] {'?'};
		}
	}
}
