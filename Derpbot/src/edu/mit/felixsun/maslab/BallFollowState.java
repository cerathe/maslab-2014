package edu.mit.felixsun.maslab;
import java.util.Map.Entry;

public class BallFollowState extends State {

	int forwardCountdown = 0;
	int pauseCountdown = 0;
	int ballStreak = 0;
	final double SPEED = Constants.SPEED;
	final double GAIN = Constants.SPEED;
	final int PLOW_TIME = 30;
	final int PAUSE_TIME = 20;
	public BallFollowState() {
		
	}
	
	public int step(Localization loc, Sensors sensors, int color) {
		/*
		 * Returns 1 if currently ball-following; 0 if no ball to follow.
		 * Color: 1=green, 2=red
		 */
		
		if (ballStreak > 150) {
			// Following ball for too long!
			return 3;
		}
		Entry<Double, Double> ballPolarLoc;
		if (color == 1) {
			ballPolarLoc = loc.ballGreenPolarLoc;
		} else {
			ballPolarLoc = loc.ballRedPolarLoc;
		}
		if (ballPolarLoc.getKey() < 0) {
			if (forwardCountdown > 0) {
				sensors.leftDriveMotor.setSpeed(SPEED);
				sensors.rightDriveMotor.setSpeed(-SPEED);
				forwardCountdown--;
				ballStreak++;
				return 1;
			} else if (pauseCountdown > 0) {
				sensors.leftDriveMotor.setSpeed(0);
				sensors.rightDriveMotor.setSpeed(0);
				loc.stuck = false;
				pauseCountdown--;
				return 1;
			}
			
			
			// No balls to follow.
			sensors.leftDriveMotor.setSpeed(0);
			sensors.rightDriveMotor.setSpeed(0);
			ballStreak = 0;
			return 0;
		}
		
		// Follow the ball.
		double angle = ballPolarLoc.getValue();
		double diff = (angle - Math.PI/2) * GAIN;
//		System.out.println(angle);
		sensors.leftDriveMotor.setSpeed(SPEED - diff);
		sensors.rightDriveMotor.setSpeed(-SPEED - diff); 
		forwardCountdown = PLOW_TIME;
		pauseCountdown = PAUSE_TIME;
		ballStreak++;
		return 1;
	}
}
