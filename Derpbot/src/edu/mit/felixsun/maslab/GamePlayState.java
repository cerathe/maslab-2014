package edu.mit.felixsun.maslab;

import edu.mit.felixsun.maslab.SparseGrid.PointOfInterest;

public class GamePlayState extends State{
	/*
	 * The top-level class that plays the game.
	 */
	long endTime;
	int scoringAttempts = 0;
	boolean goalSet = false;
	BallCollectState ball;
	DriveGoalState goal;
	public GamePlayState(Navigation nav) {
		endTime = System.nanoTime() + (long)(3) * 60 * 1000000000;
		System.out.println(System.nanoTime());
		System.out.println(endTime);
		ball = new BallCollectState(nav);
		goal = new DriveGoalState(Constants.SPEED, nav);
	}
	
	public int step(Navigation nav, Sensors sensors) {
		/*
		 * Return codes:
		 * 0 = running
		 * 1 = stopped
		 */

		if (System.nanoTime() > endTime) {
			// Time's up - stop.
			sensors.leftDriveMotor.setSpeed(0);
			sensors.rightDriveMotor.setSpeed(0);
			return 1;
		}
		if (System.nanoTime() > getTime(10) && scoringAttempts == 0) {
			// Try to dump
			System.out.println("Trying to score");
			if (!goalSet) {
				goal.setGoal(getClosestGoal(nav, 3));
				goalSet = true;
			}
			int outCode = goal.step(sensors);
			if (outCode == 2 || System.nanoTime() > getTime(100)) {
				// We either scored, or we gave up trying.
				scoringAttempts = 1;
				goalSet = false;
			}
			return 0;
		}

		ball.step(nav, sensors);
		return 0;
	}
	
	PointOfInterest getClosestGoal(Navigation nav, int goalType) {
		/*
		 * Find the closest goal.  In the future, we may want to find the closest unscored goal.
		 */
		PointOfInterest bestGoal = null;
		double bestDist = 1000000;
		for (PointOfInterest point : nav.loc.grid.places) {
			if (point.type != goalType) {
				// Not a reactor.
				continue;
			}
			double thisDist = nav.loc.grid.dist(point.actualMidpoint.getKey(), point.actualMidpoint.getValue(), 
					nav.loc.grid.robotX, nav.loc.grid.robotY);
			if (thisDist < bestDist) {
				bestGoal = point;
				bestDist = thisDist;
			}
		}
		return bestGoal;
	}
	
	long getTime(int seconds) {
		long out = endTime - (long)(180 - seconds) * 1000000000;
		return out;
	}
}