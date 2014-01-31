package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;
import java.util.Map.Entry;
import java.util.Iterator;
import java.util.LinkedList;

import edu.mit.felixsun.maslab.SparseGrid.PointOfInterest;

public class DriveGoalState extends State{
	PathFollowState pfs;
	TurnState ts = new TurnState();
	DriveStraightState deadReckon = new DriveStraightState();
	PointTrackState pointTrack = new PointTrackState(Constants.SPEED);
	final int MAX_TURN_TIME = 100;
	int substate = 0;
	int deadReckonCount = 0;
	int dumpCount = 0;
	int backCount = 0;
	int turnTimer = 0;
	public Navigation nav;
	public double finalAngle;
	public double speed;
	public LinkedList<SimpleEntry<Integer,Integer>> path;
	PointOfInterest goal;
	
	public DriveGoalState(double spd, Navigation n ){
		speed = spd;
		nav = n;
	}
	
	public void setGoal(PointOfInterest pt){
		//Check which of the two potential normal pts is reachable.
		goal = pt;
		SimpleEntry<Integer,Integer> curr ;
		try{
			curr = new SimpleEntry<Integer,Integer>((int) nav.loc.grid.robotX, (int) nav.loc.grid.robotY);
		}
		catch(NullPointerException e){
			curr = new SimpleEntry<Integer,Integer>((int) nav.loc.grid.theMap.startPose.x, (int) nav.loc.grid.theMap.startPose.y);
			
		}
		SimpleEntry<Integer,Integer> n1 = pt.normPt1;
		SimpleEntry<Integer,Integer> n2 = pt.normPt2;
		if(nav.loc.grid.accessibleArea.containsKey(n1)){
			path = nav.cleanUpNaive(nav.naiveWallFollow(curr, n1));
		}
		else{
			path = nav.cleanUpNaive(nav.naiveWallFollow(curr, n2));
		}
		pfs = new PathFollowState(speed, path);
		substate = 0;
	}
	
	public int step(Sensors s){
		/*
		 * Return codes:
		 * 0 - No path to goal
		 * 1 - Nav-ing
		 * 2 - Done
		 * 3 - Stuck
		 */
		double ANGLE_TOLERANCE = 0.1;
		if (substate == 0) {
			System.out.println("Driving towards waypoint");
			// Drive towards waypoint
			if(path==null){
				return 0;
			}
			if (nav.loc.goalDistance < 12) {
				// Ram the goal directly.
				substate = 2;
				deadReckonCount = 50;
				return 1;
			}
			
			else if(pfs==null){
				pfs = new PathFollowState(speed, path);
				pfs.step(nav, s);
				return 1;
			}
			else{
				//If you've reached the point, turn to face it.
				SimpleEntry<Integer, Integer> robotLoc = 
						new SimpleEntry<Integer, Integer>((int) nav.loc.grid.robotX, (int) nav.loc.grid.robotY);
				if (nav.loc.grid.dist(path.peekLast(), robotLoc) < 3){
					s.leftDriveMotor.setSpeed(0);
					s.rightDriveMotor.setSpeed(0);
					substate = 1;
					turnTimer = MAX_TURN_TIME;
					return 1;
				}
				else{
					int pathstate = pfs.step(nav,s);
					if(pathstate==2){
						return 3;
					}
					return 1;
				}
			}
		} else if (substate == 1) {
			if (turnTimer == 0) {
				// That didn't work :(
				return 3;
			}
			turnTimer--;
			System.out.println("Turning towards goal");
			// Turn towards goal
			double xDiff = goal.actualMidpoint.getKey() - nav.loc.grid.robotX;
			double yDiff = goal.actualMidpoint.getValue() - nav.loc.grid.robotY;
			double pathAngle = Math.atan2(yDiff, xDiff);
			double angleDiff = pathAngle - nav.loc.grid.robotTheta;
			angleDiff = Math.atan2(Math.sin(angleDiff), Math.cos(angleDiff));
			if (angleDiff < -ANGLE_TOLERANCE){
//				System.out.println("Turn A");
				ts.step(nav.loc, s, -5 * Constants.SPEED);
				return 1;
			} else if (angleDiff > ANGLE_TOLERANCE) {
//				System.out.println("Turn B");
				ts.step(nav.loc, s, 5 * Constants.SPEED);
				return 1;
			} else {
				substate = 2;
				s.leftDriveMotor.setSpeed(0);
				s.rightDriveMotor.setSpeed(0);
				deadReckonCount = 50;
				return 1;
			}
		} else if (substate == 2) {
			// Dead-reckon into goal.
			double thisSpeed = 0.6 * Constants.SPEED;
			double GOAL_GAIN = 0.8 * thisSpeed;
			System.out.println("Ramming goal.");
			if (deadReckonCount > 0) {
				if (nav.loc.goalAngle == 0) {
					s.leftDriveMotor.setSpeed(thisSpeed);
					s.rightDriveMotor.setSpeed(-thisSpeed);
				} else {
					double deltaSpeed = (nav.loc.goalAngle - Math.PI/2) * GOAL_GAIN;
					s.leftDriveMotor.setSpeed(thisSpeed - deltaSpeed);
					s.rightDriveMotor.setSpeed(-thisSpeed - deltaSpeed);
				}
				deadReckonCount--;
				return 1;
			} else {
				if (nav.loc.goalAngle == 0 || Math.abs(nav.loc.goalAngle - Math.PI/2) < 0.1) {
					substate = 3;
					s.leftDriveMotor.setSpeed(0);
					s.rightDriveMotor.setSpeed(0);
					s.rightDump.setAngle(s.rightDump.getMaxAngle());
					s.leftDump.setAngle(s.leftDump.getMinAngle());
					dumpCount = 40;
					return 1;
				} else {
					// We are probably stuck.
					return 3;
				}
			}
		} else if (substate == 3) {
			System.out.println("Dumping");
			if (dumpCount > 0) {
				dumpCount--;
				return 1;
			} else {
				s.rightDump.setAngle(s.rightDump.getMinAngle());
				s.leftDump.setAngle(s.leftDump.getMaxAngle());
				backCount = 50;
				substate = 4;
				return 1;
			}
		} else if (substate == 4) {
			System.out.println("Backing up");
			// Back out.
			if (backCount > 0) {
				deadReckon.step(nav.loc, s, -8);
				backCount--;
				return 1;
			} else {
				substate = 5;
				s.leftDriveMotor.setSpeed(0);
				s.rightDriveMotor.setSpeed(0);
				return 1;				
			}
		} 
		return 2;
	}
}
