package edu.mit.felixsun.maslab;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.Map.Entry;
import java.util.AbstractMap.SimpleEntry;

import comm.BotClientMap;
import comm.BotClientMap.Pose;

public class Localization {
	public final static int PARTICLE_COUNT = 30; 	// How many samples of the world?
	public final static int PRUNED_COUNT = 15;		// How many samples do we keep at the end of each step?
	public final static double TRAVEL_DRIFT_SPEED = 5;			// Inches / second
	public final static double TURN_DRIFT_SPEED = 1;			// Radians / second
	// How uncertain are we about our starting location?
	public final static double INITIAL_DELTA_LOC = 2;
	public final static double INITIAL_DELTA_ANGLE = 0.2;
	public final static double STUCK_VEL = 1;		// Inches/second
	
	BotClientMap map;
	public SparseGrid grid;
	public List<Pose> robotPositions;
	public Entry<Double, Double> ballGreenPolarLoc;	// Just pass this on for now.
	public Entry<Double, Double> ballRedPolarLoc;
	public double forwardSpeed;
	public double turnSpeed;
	public boolean stuck;
	public boolean relocalize = false;		// If this is set to true, we start from scratch and relocalize.
	public double goalAngle = 0;
	public double goalDistance = 1000;
	int initialCountdown = 20;
	List<Double> initialProbs = new ArrayList<Double>();
	double meanProb;
	double sdProb;
	double normalization;
	long lastUpdateTime;
	int stuckCount;
	Random rng;

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
	
	public Localization(cvData data, BotClientMap map) {
		this.map = map;
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
	
	void relocalize(cvData data) {
		/*
		 * Rebuilds the localization data from scratch.  Call this if we get really lost.
		 * Kind of computationally intensive.
		 */
		System.out.println("Reloc");
		int RELOCALIZE_PARTICLE_COUNT = 5000;
		ArrayList<Pose> hypotheses = new ArrayList<Pose>();
		// Make a whole bunch of hypotheses.
		for (int i=0; i<RELOCALIZE_PARTICLE_COUNT; i++) {
			double angle = 2*Math.PI*rng.nextDouble() - Math.PI;
			double x = rng.nextDouble() * grid.maxX;
			double y = rng.nextDouble() * grid.maxY;
			// First of all, is it actually on the field?
			Entry<Integer, Integer> point = new SimpleEntry<Integer, Integer>(
					(int)x, (int)y);
			if (!grid.accessibleArea.containsKey(point)) {
				continue;
			}
			double prob = grid.stateLogProb(data, x, y, angle);
			hypotheses.add(new Pose(x, y, angle, prob));
		}
		Collections.sort(hypotheses, new ProbCompare());
		robotPositions = hypotheses.subList(0, PARTICLE_COUNT);
		Pose bestGuess = robotPositions.get(0);
		grid.robotX = bestGuess.x;
		grid.robotY = bestGuess.y;
		grid.robotTheta = bestGuess.theta;
		relocalize = false;
	}
	
	public void update(cvData data, Sensors sensors) {
		
		this.ballGreenPolarLoc = data.ballGreenPolarLoc;
		this.ballRedPolarLoc = data.ballRedPolarLoc;
		List<Entry<Double,Double>> greenGoals = data.landmarks.get(0);
		if (greenGoals != null && greenGoals.size() > 0) {
			goalAngle = 0;
			goalDistance = 72;
			for (Entry<Double, Double> goalPoint : greenGoals) {
				double dist = goalPoint.getKey();
				if (dist < 48 && dist < goalDistance) {
					goalAngle = goalPoint.getValue();
					goalDistance = dist;
				}
			}
		} else {
			goalAngle = 0;
		}
		if (data.angles.size() == 0) {
			return;
		}
		
		if (relocalize) {
			relocalize(data);
			relocalize = false;
			return;
		}
		
		double deltaT = (double) (System.nanoTime() - lastUpdateTime) / 1000000000;
		lastUpdateTime = System.nanoTime();
		double realRightEncoder = sensors.rightEncoder.getDeltaAngularDistance();
		// Calculate drift using encoders.
		double deltaLeft = -sensors.leftEncoder.getDeltaAngularDistance() * Constants.WHEEL_RADIUS;
		double deltaRight = realRightEncoder * Constants.WHEEL_RADIUS;
		double forward = (deltaLeft + deltaRight) / 2;
		double turn = (deltaRight - deltaLeft) / Constants.WHEELBASE_WIDTH;
		forwardSpeed = forward / deltaT;
		turnSpeed = turn / deltaT;

		if ((Math.abs(sensors.leftDriveMotor.lastSet) > 0.01 || 
				Math.abs(sensors.rightDriveMotor.lastSet) > 0.01) &&
				Math.abs(deltaLeft) < STUCK_VEL*deltaT &&
				Math.abs(deltaRight) < STUCK_VEL*deltaT) {
			stuckCount++;
		} else {
			stuckCount = 0;
		}
		if (stuckCount > 30) {
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
				newProb = grid.stateLogProb(data, newX, newY, newTheta) + oldPose.prob - normalization;
			}
			robotPositions.set(i, new Pose(newX, newY, newTheta, newProb));
		}
		
		// Delete the unlikely particles.  Clone the likely particles.
		Collections.sort(robotPositions, new ProbCompare());
		for (int i=PRUNED_COUNT; i<PARTICLE_COUNT; i++) {
			robotPositions.set(i, robotPositions.get(i % PRUNED_COUNT));
		}
		Pose bestGuess = robotPositions.get(0);
		normalization = bestGuess.prob;
		// Update the average prob.
		if (initialCountdown > 0) {
			initialProbs.add(normalization);
			initialCountdown--;
			if (initialCountdown == 0) {
				// Calculate mean and SD
				meanProb = mean(initialProbs);
				sdProb = standardDeviation(initialProbs);
				System.out.format("Mean: %f SD: %f", meanProb, sdProb);
			}
		} else {
			if (normalization < meanProb - 3*sdProb) {
				// OK. this isn't right.  Relocalize now!
				//relocalize = true;
			}
		}
		grid.robotX = bestGuess.x;
		grid.robotY = bestGuess.y;
		grid.robotTheta = bestGuess.theta;
	}
	
	double mean(List<Double> numbers) {
		double sum = 0;
		for(double number : numbers) {
			sum += number;
		}
		return sum / numbers.size();
	}
	
	double standardDeviation(List<Double> numbers) {
		double var = 0;
		double mean = mean(numbers);
		for(double number : numbers) {
			var += Math.pow((number - mean), 2);
		}
		return Math.pow(var, 0.5);
	}
}
