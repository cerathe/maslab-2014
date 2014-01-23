package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;

import comm.BotClientMap.Pose;

public class PointTrackState extends State{
	TurnState turnState = new TurnState();
	double lastDiff;
	
	double acceptableAngle = 0.05;
	double driveSpeed;
	double PGAIN = 0.1;
	
	public PointTrackState(double speed){
		lastDiff = -5; //dummy value
		driveSpeed = speed;
	}
	
	public void step(Navigation nav, Sensors sensors, SimpleEntry<Integer,Integer> pt){
		Pose currentPose = new Pose(nav.loc.grid.robotX, nav.loc.grid.robotY, nav.loc.grid.robotTheta);
		double xDiff = nav.loc.grid.robotX - pt.getKey();
		double yDiff = nav.loc.grid.robotY - pt.getValue();
		double pathAngle = Math.atan(yDiff/xDiff);
		if(xDiff<0){
			pathAngle += Math.PI;
		}
		double angleDiff = pathAngle - currentPose.theta;
		if(angleDiff>Math.PI){
			angleDiff = 2*Math.PI - angleDiff;
		}
		else if(angleDiff<-Math.PI){
			angleDiff = angleDiff + 2*Math.PI;
		}
		
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
		if(Math.abs(angleDiff)<acceptableAngle){
			motorA = driveSpeed;
			motorB = driveSpeed;
			sensors.leftDriveMotor.setSpeed(-motorA);
			sensors.rightDriveMotor.setSpeed(motorB);			
		}
		else{
			turnState.step(nav.loc, sensors, -angleDiff/2*Math.PI * PGAIN);
		}
		
		
	}
	
}
