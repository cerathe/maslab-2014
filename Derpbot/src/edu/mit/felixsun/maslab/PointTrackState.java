package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;

import comm.BotClientMap.Pose;

public class PointTrackState extends State{
	TurnState turnState = new TurnState();
	double lastDiff;
	
	double acceptableAngle = 0.3;
	double driveSpeed;
	double PGAIN = .1;

	
	public PointTrackState(double speed){
		lastDiff = -5; //dummy value
		driveSpeed = speed;
	}
	
	public void step(Navigation nav, Sensors sensors, SimpleEntry<Integer,Integer> pt){
		Pose currentPose = new Pose(nav.loc.grid.robotX, nav.loc.grid.robotY, nav.loc.grid.robotTheta);
		double xDiff = pt.getKey() - nav.loc.grid.robotX;
		double yDiff = pt.getValue() - nav.loc.grid.robotY;
//		System.out.format("%f %f \n", xDiff, yDiff);
		double pathAngle = Math.atan2(yDiff, xDiff);
		double angleDiff = pathAngle - currentPose.theta;
		angleDiff = Math.atan2(Math.sin(angleDiff), Math.cos(angleDiff));
		
//		double derivative;
//		if(lastDiff==-5){
//			derivative=0;
//		}
//		else{
//			derivative = angleDiff - lastDiff;
//		}
//		
//		double integral;

		double motorA, motorB;
//		System.out.println(angleDiff);
		if(Math.abs(angleDiff)<acceptableAngle){
			motorA = driveSpeed - angleDiff * PGAIN;
			motorB = driveSpeed + angleDiff * PGAIN;
			sensors.leftDriveMotor.setSpeed(motorA);
			sensors.rightDriveMotor.setSpeed(-motorB);
		}
		else if (angleDiff < 0){
//			System.out.println("Turn A");
			turnState.step(nav.loc, sensors, 0.2);
		} else {
//			System.out.println("Turn B");
			turnState.step(nav.loc, sensors, -0.2);
		}
		
		
	}
	
}
