package edu.mit.felixsun.maslab;

public class DriveStraightState extends State {
	double lastLeft = 0;
	double lastRight = 0;
	double lastSpeed = 0;
	final double FORWARD_GAIN = 0.002;
	final double TURN_GAIN = 0.01;
	public DriveStraightState() {
	}
	
	public int step(Localization loc, Sensors sensors, double speed) {
		/*
		 * Drives straight forward at speed inches / sec
		 * Always returns 0.
		 * Uses left motor to control speed, right motor to control heading.
		 */
		double leftMotor = 0;
		double rightMotor = 0;
		if (lastSpeed != speed) {
			// New problem.  (Not sure what to do here)
			leftMotor = 0;
			rightMotor = 0;
			lastSpeed = speed;
			
		} else {
			double deltaForward = FORWARD_GAIN * (speed - loc.forwardSpeed);
			double deltaTurn = TURN_GAIN * loc.turnSpeed;
			rightMotor = lastRight + deltaForward - deltaTurn;
			leftMotor = lastLeft + deltaForward + deltaTurn;
		}
		lastLeft = leftMotor;
		lastRight = rightMotor;
		sensors.leftDriveMotor.setSpeed(leftMotor);
		sensors.rightDriveMotor.setSpeed(-rightMotor);
		
		return 0;
	}
}
