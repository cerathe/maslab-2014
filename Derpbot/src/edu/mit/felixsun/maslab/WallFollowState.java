package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;

public class WallFollowState extends State {

	double lastD;
	double setPoint;
	double driveSpeed;
	double PGAIN = -0.01;	// Motor units / inch
	double DGAIN = -0.05;
	int stuckCount = 0;
	TurnState turnState = new TurnState();
	public WallFollowState(double dist, double speed) {
		lastD = -1.23;	// An initial value, to indicate that we don't have prior state.
		if (dist < 0) {
			setPoint = 4;
		} else {
			setPoint = dist;
		}
		if (speed < 0) {
			driveSpeed = 0.1;
		} else {
			driveSpeed = speed;
		}
	}
	
	public void step(Localization loc, Sensors sensors) {
		
		double leftD = getSideDistance(loc.grid, -1);
		double rightD = getSideDistance(loc.grid, 1);
//		System.out.format("%f %f \n", leftD, rightD);
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
		double diff = PGAIN*(minD - setPoint - Constants.ROBOT_WIDTH/2) + DGAIN*derivative;
		
		double motorA, motorB;
		double frontDist = getFrontDistance(loc.grid, Constants.ROBOT_WIDTH);
		if (frontDist < setPoint) {
//			if (direction == 1) {
				System.out.println("Too close, turn left");
				turnState.step(loc, sensors, -0.1);
//			} else {
//				System.out.println("Too close, turn right");
//				turnState.step(loc, sensors, 0.1);
//			}
		} else {
			if (direction == -1) {
				System.out.println("Follow left");
				motorA = driveSpeed + diff;
				motorB = driveSpeed - diff;
			} else {
				System.out.println("Follow right");
				motorA = driveSpeed - diff;
				motorB = driveSpeed + diff;
			}
			sensors.leftDriveMotor.setSpeed(-motorA);
			sensors.rightDriveMotor.setSpeed(motorB);
		}
	}
	
	double getSideDistance(SparseGrid map, int direction) {
		/*
		 * Find the distance from the robot to a side wall.
		 * direction:
		 * -1 = left
		 * 1 = right
		 */
		double angle;
		if (direction == 1) {
			angle = 0;
		} else {
			angle = Math.PI;
		}
		double dist = map.trueMeas(angle, map.robotX, map.robotY, map.robotTheta, 48);
		if (dist < 0) {
			return 24;
		}
		return dist - Constants.ROBOT_WIDTH/2;
	}
	
	double getFrontDistance(SparseGrid map, double robotWidth) {
		/*
		 * Finds the distance from the front of the robot to a wall.
		 */
		double dist = map.trueMeas(Math.PI/2, map.robotX, map.robotY, map.robotTheta, 48);
		if (dist < 0) {
			return 24;
		}
		return dist - Constants.ROBOT_WIDTH/2;
	}
	
}
