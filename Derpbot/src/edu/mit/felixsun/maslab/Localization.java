package edu.mit.felixsun.maslab;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Random;

import comm.BotClientMap;
import comm.BotClientMap.Pose;

public class Localization {
	public final static int PARTICLE_COUNT = 10; 	// How many samples of the world?
	public final static int PRUNED_COUNT = 5;		// How many samples do we keep at the end of each step?
	public final static double TRAVEL_DRIFT_SPEED = 0.05;		// Inches / second
	public final static double TURN_DRIFT_SPEED = 0.03;			// Radians / second
	public final static double WHEEL_RADIUS = 1.875;
	
	BotClientMap map;
	public SparseGrid grid;
	public ArrayList<Pose> robotPositions;
	double normalization;
	long lastUpdateTime;
	Random rng;

	
	public Localization(cvData data) {
		map = BotClientMap.getDefaultMap();
		grid = new SparseGrid(data.gridSize, map, data.robotWidth);
		robotPositions = new ArrayList<Pose>();
		Pose startPose = new Pose(grid.robotX, grid.robotY, grid.robotTheta);
		for (int i=0; i<PARTICLE_COUNT; i++) {
			robotPositions.add(startPose);
		}
		normalization = 0;
		grid.writeMap();
		rng = new Random();
		lastUpdateTime = System.nanoTime();
	}
	
	public void update(cvData data, Sensors sensors) {
		if (data.angles.size() == 0) {
			return;
		}
		double deltaT = (System.nanoTime() - lastUpdateTime) / 1000000000;
		lastUpdateTime = System.nanoTime();
		
		// Calculate drift using encoders.
		double deltaLeft = sensors.leftEncoder.getDeltaAngularDistance() * WHEEL_RADIUS;
		double deltaRight = sensors.rightEncoder.getDeltaAngularDistance() * WHEEL_RADIUS;
		double forward = (deltaLeft + deltaRight) / 2;
		double turn = (deltaRight - deltaLeft) / data.robotWidth;
		
		// Update each particle with the expected drift.
		// Right now, the drift is Gaussian, but this can be a lot better, with encoder data.
		// Update the probability of each particle.
		for (int i=0; i<PARTICLE_COUNT; i++) {
			Pose oldPose = robotPositions.get(i);
			double wheelDeltaX = forward * Math.cos(oldPose.theta);
			double wheelDeltaY = forward * Math.sin(oldPose.theta);
			double newX = oldPose.x + rng.nextGaussian() * TRAVEL_DRIFT_SPEED * deltaT + wheelDeltaX;
			double newY = oldPose.y + rng.nextGaussian() * TRAVEL_DRIFT_SPEED * deltaT + wheelDeltaY;
			double newTheta = oldPose.theta + rng.nextGaussian() * TURN_DRIFT_SPEED * deltaT + turn;
			double newProb = grid.stateLogProb(data.angles, newX, newY, newTheta) + oldPose.prob - normalization;
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
		System.out.println(normalization);
		grid.robotX = bestGuess.x;
		grid.robotY = bestGuess.y;
		grid.robotTheta = bestGuess.theta;
	}
}
