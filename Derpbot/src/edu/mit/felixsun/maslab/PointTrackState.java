package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;

import comm.BotClientMap.Pose;

public class PointTrackState extends State{
	double tolerance = 5;
	double lowestSpeed = 3; // in/s
	
	TurnState turnState = new TurnState();
	double lastDiff;
	
	double acceptableAngle = 0.5;
	double driveSpeed;
	double PGAIN = .05;
	long expectedTime = -1;
	long startTime;

	
	public PointTrackState(double speed){
		lastDiff = -5; //dummy value
		driveSpeed = speed;
	}
	
	public int step(Navigation nav, Sensors sensors, SimpleEntry<Integer,Integer> pt){
		Pose currentPose = new Pose(nav.loc.grid.robotX, nav.loc.grid.robotY, nav.loc.grid.robotTheta);
		double xDiff = pt.getKey() - nav.loc.grid.robotX;
		double yDiff = pt.getValue() - nav.loc.grid.robotY;
		System.out.format("Goal: %d %d Current: %f %f \n", pt.getKey(), pt.getValue(), currentPose.x, currentPose.y);
		double pathAngle = Math.atan2(yDiff, xDiff);
		double angleDiff = pathAngle - currentPose.theta;
		angleDiff = Math.atan2(Math.sin(angleDiff), Math.cos(angleDiff));
		double dist = Math.sqrt(xDiff*xDiff + yDiff*yDiff);
		
		if(expectedTime==-1){
			expectedTime = (long)(dist/lowestSpeed)*(long)(1000);
			startTime = System.currentTimeMillis();
		}

		double motorA, motorB;
		if(dist<tolerance){
			expectedTime = -1;
			return 1;
		}
//		System.out.println(angleDiff);
		if(System.currentTimeMillis() - startTime>expectedTime){
			return 2;
		}
		if(Math.abs(angleDiff)<acceptableAngle){
			motorA = driveSpeed - angleDiff * PGAIN;
			motorB = driveSpeed + angleDiff * PGAIN;
			System.out.format("Motor speeds: %f, %f", motorA, motorB);
			sensors.leftDriveMotor.setSpeed(motorA);
			sensors.rightDriveMotor.setSpeed(-motorB);
		}
		else if (angleDiff < 0){
//			System.out.println("Turn A");
			turnState.step(nav.loc, sensors, -13 * driveSpeed);
		} else {
//			System.out.println("Turn B");
			turnState.step(nav.loc, sensors, 13 * driveSpeed);
		}
//		nav.loc.grid.safeSet(pt.getKey(),pt.getValue(),1);
		return 0;
		
	}
	
}
