package edu.mit.felixsun.maslab;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Random;
import java.util.Map.Entry;

import comm.BotClientMap;
import comm.BotClientMap.Pose;

public class Localization {
	public final static int PARTICLE_COUNT = 30; 	// How many samples of the world?
	public final static int PRUNED_COUNT = 15;		// How many samples do we keep at the end of each step?
	public final static double TRAVEL_DRIFT_SPEED = 3;			// Inches / second
	public final static double TURN_DRIFT_SPEED = 0.3;			// Radians / second
	// How uncertain are we about our starting location?
	public final static double INITIAL_DELTA_LOC = 2;
	public final static double INITIAL_DELTA_ANGLE = 0.02;
	public final static double STUCK_VEL = 0.1;		// Inches/second
	
	BotClientMap map;
	public SparseGrid grid;
	public ArrayList<Pose> robotPositions;
	public Entry<Double, Double> ballPolarLoc;	// Just pass this on for now.
	public double forwardSpeed;
	public double turnSpeed;
	public boolean stuck;
	double normalization;
	long lastUpdateTime;
	int stuckCount;
	Random rng;

	
	public Localization(cvData data) {
		map = BotClientMap.getDefaultMap();
		grid = new SparseGrid(data.gridSize, map);
		forwardSpeed = 0;
		turnSpeed = 0;
		stuck = false;
		stuckCount = 0;
		rng = new Random();
		robotPositions = new ArrayList<Pose>();
		for (int i=0; i<PARTICLE_COUNT; i++) {
			Pose startPose = new Pose(grid.robotX + rng.nextGaussian() * INITIAL_DELTA_LOC,
					grid.robotY + rng.nextGaussian() * INITIAL_DELTA_LOC, 
					grid.robotTheta + rng.nextGaussian() * INITIAL_DELTA_ANGLE);
			robotPositions.add(startPose);
		}
		normalization = 0;
		grid.writeMap();
		lastUpdateTime = System.nanoTime();
	}
	
	public void update(cvData data, Sensors sensors) {
		
		this.ballPolarLoc = data.ballPolarLoc;
		if (data.angles.size() == 0) {
			return;
		}
		double deltaT = (double) (System.nanoTime() - lastUpdateTime) / 1000000000;
		lastUpdateTime = System.nanoTime();
		
		// Calculate drift using encoders.
		double deltaLeft = -sensors.leftEncoder.getDeltaAngularDistance() * Constants.WHEEL_RADIUS;
		double deltaRight = sensors.rightEncoder.getDeltaAngularDistance() * Constants.WHEEL_RADIUS;
		double forward = (deltaLeft + deltaRight) / 2;
		double turn = (deltaRight - deltaLeft) / Constants.WHEELBASE_WIDTH;
		forwardSpeed = forward / deltaT;
		turnSpeed = turn / deltaT;

		// Are we stuck?
		if ((Math.abs(sensors.leftDriveMotor.lastSet) > 0.01 || 
				Math.abs(sensors.rightDriveMotor.lastSet) > 0.01) &&
				Math.abs(deltaLeft) < STUCK_VEL*deltaT &&
				Math.abs(deltaRight) < STUCK_VEL*deltaT) {
			stuckCount++;
		} else {
			stuckCount = 0;
		}
		if (stuckCount > 40) {
			stuck = true;
			System.out.println("Oh fuck, we're stuck.");
		} else {
			stuck = false;
		}
		
		// Update each particle with the expected drift.
		// Update the probability of each particle.
		for (int i=0; i<PARTICLE_COUNT; i++) {
			Pose oldPose = robotPositions.get(i);
			double wheelDeltaX = forward * Math.cos(oldPose.theta);
			double wheelDeltaY = forward * Math.sin(oldPose.theta);
			double newX = oldPose.x + rng.nextGaussian() * TRAVEL_DRIFT_SPEED * deltaT + wheelDeltaX;
			double newY = oldPose.y + rng.nextGaussian() * TRAVEL_DRIFT_SPEED * deltaT + wheelDeltaY;
			double newTheta = oldPose.theta + rng.nextGaussian() * TURN_DRIFT_SPEED * deltaT + turn;
			if (newTheta > Math.PI) {
				newTheta -= Math.PI*2;
			} else if (newTheta < -Math.PI) {
				newTheta += Math.PI*2;
			}
			
			double newProb;
			// If we are stuck, we must be next to a wall.
			if (stuck && grid.closestOccupied(newX, newY) > Constants.ROBOT_WIDTH / 2 + 1) {
				newProb = -100000000;
			} else {
				newProb = grid.stateLogProb(data.angles, newX, newY, newTheta) + oldPose.prob - normalization;
			}
			robotPositions.set(i, new Pose(newX, newY, newTheta, newProb));
		}
		
		// Delete the unlikely particles.  Clone the likely particles.
		class ProbCompare implements Comparator<Pose> {
			public int compare(Pose a, Pose b) {
				if (a.prob > b.prob) {
					return -1;
				} else if (a.prob < b.prob) {
					return 1;
				} else {
					return 0;
				}
			}
		}
		Collections.sort(robotPositions, new ProbCompare());
		for (int i=PRUNED_COUNT; i<PARTICLE_COUNT; i++) {
			robotPositions.set(i, robotPositions.get(i % PRUNED_COUNT));
		}
		Pose bestGuess = robotPositions.get(0);
		normalization = bestGuess.prob;
//		System.out.println(normalization);
		grid.robotX = bestGuess.x;
		grid.robotY = bestGuess.y;
		grid.robotTheta = bestGuess.theta;
	}
}
