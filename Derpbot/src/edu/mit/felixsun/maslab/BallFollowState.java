package edu.mit.felixsun.maslab;

public class BallFollowState extends State {

	int forwardCountdown = 0;
	final double SPEED = .1;
	final double GAIN = .1;
	final int PLOW_TIME = 10;
	public BallFollowState() {
		
	}
	
	public int step(Localization loc, Sensors sensors) {
		/*
		 * Returns 1 if currently ball-following; 0 if no ball to follow.
		 */
		
		if (loc.ballPolarLoc.getKey() < 0) {
			if (forwardCountdown > 0) {
				sensors.leftDriveMotor.setSpeed(SPEED);
				sensors.rightDriveMotor.setSpeed(-SPEED);
				forwardCountdown--;
				return 1;
			}
			// No balls to follow.
			sensors.leftDriveMotor.setSpeed(0);
			sensors.rightDriveMotor.setSpeed(0);
			return 0;
		}
		
		// Follow the ball.
		double angle = loc.ballPolarLoc.getValue();
		double diff = (angle - Math.PI/2) * GAIN;
//		System.out.println(angle);
		sensors.leftDriveMotor.setSpeed(SPEED - diff);
		sensors.rightDriveMotor.setSpeed(-SPEED - diff); 
		forwardCountdown = PLOW_TIME;
		return 1;
	}
}
