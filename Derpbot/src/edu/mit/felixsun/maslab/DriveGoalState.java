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
	int substate = 0;
	int deadReckonCount = 0;
	int dumpCount = 0;
	int backCount = 0;
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
		System.out.println(n1 + " " +n2);
		LinkedList<SimpleEntry<Integer, Integer>> p1 = nav.naiveWallFollow(curr,n1);
		LinkedList<SimpleEntry<Integer, Integer>> p2 = nav.naiveWallFollow(curr, n2);
		System.out.println("p1: "+p1);
		//Give the path to follow.
		if(nav.loc.grid.getNeighbors(p1.peekLast()).contains(n1) || p1.peekLast().equals(n1)){
			System.out.println("1! "+p1.peekLast());
			path = nav.cleanUpNaive(p1);
			double xdiff = pt.actualMidpoint.getKey() - pt.normPt1.getKey();
			double ydiff = pt.actualMidpoint.getValue() - pt.normPt1.getValue();
			finalAngle = Math.atan2(xdiff, ydiff);
		}
		else if(nav.loc.grid.getNeighbors(p2.peekLast()).contains(n2) || p2.peekLast().equals(n2)){
			System.out.println("2!");
			path = nav.cleanUpNaive(p2);
			double xdiff = pt.actualMidpoint.getKey() - pt.normPt2.getKey();
			double ydiff = pt.actualMidpoint.getValue() - pt.normPt2.getValue();
			finalAngle = Math.atan2(xdiff, ydiff);
		}
		else{
			System.out.println("Couldn't reach this goal at " + pt.actualMidpoint);
		}
	}
	
	public int step(Sensors s){
		/*
		 * Return codes:
		 * 0 - No path to goal
		 * 1 - Nav-ing
		 * 2 - Done
		 */
		double ANGLE_TOLERANCE = 0.05;
		if (substate == 0) {
			System.out.println("Driving towards waypoint");
			// Drive towards waypoint
			if(path==null){
				return 0;
			}
			if(deadReckonCount>1){
				//backing up
				deadReckonCount--;
				pointTrack.step(nav, s, goal.actualMidpoint);
				return 1;
			}
			if (deadReckonCount==2) {
				nav.loc.relocalize = true;
				deadReckonCount--;
			}
			
			if(deadReckonCount==1){
				setGoal(goal);
				deadReckonCount--;
				return 1;
			}
			if (nav.loc.stuck) {
				// Stuck.
				deadReckonCount = 20;
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
				if (nav.loc.grid.dist(path.peekLast(), robotLoc) < 2){
					s.leftDriveMotor.setSpeed(0);
					s.rightDriveMotor.setSpeed(0);
					substate = 1;
					return 1;
				}
				else{
					pfs.step(nav,s);
					return 1;
				}
			}
		} else if (substate == 1) {
			System.out.println("Turning towards goal");
			// Turn towards goal
			double xDiff = goal.actualMidpoint.getKey() - nav.loc.grid.robotX;
			double yDiff = goal.actualMidpoint.getValue() - nav.loc.grid.robotY;
			double pathAngle = Math.atan2(yDiff, xDiff);
			double angleDiff = pathAngle - nav.loc.grid.robotTheta;
			angleDiff = Math.atan2(Math.sin(angleDiff), Math.cos(angleDiff));
			if (angleDiff < -ANGLE_TOLERANCE){
//				System.out.println("Turn A");
				ts.step(nav.loc, s, -13 * Constants.SPEED);
				return 1;
			} else if (angleDiff > ANGLE_TOLERANCE) {
//				System.out.println("Turn B");
				ts.step(nav.loc, s, 13 * Constants.SPEED);
				return 1;
			} else {
				substate = 2;
				s.leftDriveMotor.setSpeed(0);
				s.rightDriveMotor.setSpeed(0);
				deadReckonCount = 30;
				return 1;
			}
		} else if (substate == 2) {
			// Dead-reckon into goal.
			System.out.println("Ramming goal.");
			if (deadReckonCount > 0 && !nav.loc.stuck) {
				deadReckon.step(nav.loc, s, 18);
				deadReckonCount--;
				return 1;
			} else {
				substate = 3;
				s.leftDriveMotor.setSpeed(0);
				s.rightDriveMotor.setSpeed(0);
				s.rightDump.setAngle(s.rightDump.getMaxAngle());
				dumpCount = 40;
				return 1;
			}
		} else if (substate == 3) {
			System.out.println("Dumping");
			if (dumpCount > 0) {
				dumpCount--;
				return 1;
			} else {
				s.rightDump.setAngle(s.rightDump.getMinAngle());
				backCount = 30;
				substate = 4;
				return 1;
			}
		} else if (substate == 4) {
			System.out.println("Backing up");
			// Back out.
			if (backCount > 0) {
				deadReckon.step(nav.loc, s, -5);
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
