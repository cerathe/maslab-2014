package edu.mit.felixsun.maslab;

public class TurnState extends State {
	/*
	 * Gets the robot to turn in place.
	 */
	static final double FORWARD_GAIN = 0.01; // Motor units / (inch/second)
	static final double TURN_GAIN = 0.01;	 // Motor units / (radian/second)
	
	double lastLeft;
	double lastRight;
	double lastRate;
	public TurnState() {
		lastLeft = 0;
		lastRight = 0;
		lastRate = 0;
	}
	
	public void step(Localization loc, Sensors sensors, double rate) {
		/*
		 * rate: negative means left.
		 * rate in radians per second.
		 */
		double leftMotor = 0;
		double rightMotor = 0;
		if (lastRate != rate) {
			// New problem.  (Not sure what to do here)
			leftMotor = rate;
			rightMotor = -rate;
			lastRate = rate;
			
		} else {
			// Left motor stabilizes turn rate.
			// Right motor makes sure robot turns in place.
			rightMotor = lastRight - FORWARD_GAIN * loc.forwardSpeed;
			leftMotor = lastLeft - TURN_GAIN * (loc.turnSpeed - rate);
		}
		lastLeft = leftMotor;
		lastRight = rightMotor;
//		System.out.println(loc.forwardSpeed);
//		System.out.println(loc.turnSpeed);
		sensors.leftDriveMotor.setSpeed(-leftMotor);
		sensors.rightDriveMotor.setSpeed(rightMotor);
	}
}
