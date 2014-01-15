package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;

public class WallFollowState extends State {

	double lastD;
	double setPoint;
	double driveSpeed;
	double PGAIN = -0.01;	// Motor units / inch
	double DGAIN = -0.01;
	public WallFollowState(double dist, double speed) {
		lastD = -1.23;	// An initial value, to indicate that we don't have prior state.
		if (dist < 0) {
			setPoint = 5;
		} else {
			setPoint = dist;
		}
		if (speed < 0) {
			driveSpeed = 0.2;
		} else {
			driveSpeed = speed;
		}
	}
	
	public void step(cvData data, Sensors sensors) {
		double leftD = getSideDistance(data.grid, -1);
		double rightD = getSideDistance(data.grid, 1);
		double minD;
		int direction;	// -1 = left; 1 = right
		if (leftD < rightD) {
			minD = leftD;
			direction = -1;
		} else {
			minD = rightD;
			direction = 1;
		}
		
		double derivative;
		if (lastD == -1.23) {
			derivative = 0;
		} else {
			derivative = minD - lastD;
		}
		double diff = PGAIN*(minD - setPoint) + DGAIN*derivative;
		
		double motorA, motorB;
		if (getFrontDistance(data.grid, data.robotWidth) < setPoint) {
			motorA = -driveSpeed;
			motorB = driveSpeed;
		} else if (direction == -1) {
			motorA = driveSpeed + diff;
			motorB = driveSpeed - diff;
		} else {
			motorA = driveSpeed - diff;
			motorB = driveSpeed + diff;
		}
		
		sensors.leftDriveMotor.setSpeed(-motorA);
		sensors.rightDriveMotor.setSpeed(motorB);
	}
	
	double getSideDistance(SparseGrid map, int direction) {
		/*
		 * Find the distance from the robot to a side wall.
		 * direction:
		 * -1 = left
		 * 1 = right
		 */
		int max_len = (int) (24 / map.gridSize);
		int gridX;
		for (gridX = 0; gridX < max_len; gridX += direction) {
			SimpleEntry<Integer, Integer> coords = new SimpleEntry<Integer, Integer>(gridX, 0);
			if (map.wallNumbers.contains(map.map.get(coords))) {
				break;
			}
		}
		return Math.abs(gridX)*map.gridSize;
	}
	
	double getFrontDistance(SparseGrid map, double robotWidth) {
		/*
		 * Finds the distance from the front of the robot to a wall.
		 */
		double minDist = 24;
		for (double x = -robotWidth; x < robotWidth; x += map.gridSize) {
			double y;
			for (y = 0; y < 24; y += map.gridSize) {
				if (map.wallNumbers.contains(map.get(x, y))) {
					break;
				}
			}
			if (y < minDist) {
				minDist = y;
			}
		}
	return minDist;
	}
	
}
