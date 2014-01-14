package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;

public class WallFollowState extends State {

	double lastD;
	double setPoint;
	double driveSpeed;
	double PGAIN = -5;	// Motor units / inch
	double DGAIN = -5;
	public WallFollowState(double dist, double speed) {
		lastD = -1.23;	// An initial value, to indicate that we don't have prior state.
		if (dist < 0) {
			setPoint = 5;
		} else {
			setPoint = dist;
		}
		if (speed < 0) {
			driveSpeed = 20;
		} else {
			driveSpeed = speed;
		}
	}
	
	public byte[] step(cvData data) {
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
		
		int motorA, motorB;
		if (direction == -1) {
			motorA = (int) (driveSpeed + diff);
			motorB = (int) (driveSpeed - diff);
		} else {
			motorA = (int) (driveSpeed - diff);
			motorB = (int) (driveSpeed + diff);
		}
		
		byte[] outData = new byte[4];
		outData[0] = 'S';				// Start signal "S"
		outData[1] = (byte) -motorA;	// Motor A data
		outData[2] = (byte) motorB;		// Motor B data
		outData[3] = 'E';				// End signal "E"
		return outData;
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
	
}
