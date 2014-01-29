package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;
import java.util.Map.Entry;
import java.util.Iterator;
import java.util.LinkedList;

import edu.mit.felixsun.maslab.SparseGrid.PointOfInterest;

public class DriveGoalState extends State{
	public PathFollowState pfs;
	public TurnState ts = new TurnState();
	public Navigation nav;
	public double finalAngle;
	public double speed;
	public LinkedList<SimpleEntry<Integer,Integer>> path;
	
	public DriveGoalState(double spd, Navigation n ){
		speed = spd;
		nav = n;
	}
	
	public void setGoal(PointOfInterest pt){
		//Check which of the two potential normal pts is reachable.
		SimpleEntry<Integer,Integer> curr ;
		try{
			curr = new SimpleEntry<Integer,Integer>((int) nav.loc.grid.robotX, (int) nav.loc.grid.robotY);
		}
		catch(NullPointerException e){
			curr = new SimpleEntry<Integer,Integer>((int) nav.loc.grid.theMap.startPose.x, (int) nav.loc.grid.theMap.startPose.y);
			
		}
		SimpleEntry<Integer,Integer> n1 = pt.normPt1;
		SimpleEntry<Integer,Integer> n2 = pt.normPt2;
		LinkedList<SimpleEntry<Integer, Integer>> p1 = nav.naiveWallFollow(curr,n1);
		LinkedList<SimpleEntry<Integer, Integer>> p2 = nav.naiveWallFollow(curr, n2);
		System.out.println(p1);
		//Give the path to follow.
		if(nav.loc.grid.getNeighbors(p1.peekLast()).contains(n1)){
			System.out.println("1! "+p1.peekLast());
			path = nav.cleanUpNaive(p1);
			double xdiff = pt.actualMidpoint.getKey() - pt.normPt1.getKey();
			double ydiff = pt.actualMidpoint.getValue() - pt.normPt1.getValue();
			finalAngle = Math.atan2(xdiff, ydiff);
		}
		else if(nav.loc.grid.getNeighbors(p2.peekLast()).contains(n2)){
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
		if(path==null){
			return 0;
		}
		else if(pfs==null){
			pfs = new PathFollowState(speed/2, path);
			pfs.step(nav, s);
			return 1;
		}
		else{
			//If you've reached the point, turn to face it.
//			if(nav.loc.grid.getNeighbors(path.getLast()).contains()){
			SimpleEntry<Integer, Integer> robotLoc = 
					new SimpleEntry<Integer, Integer>((int) nav.loc.grid.robotX, (int) nav.loc.grid.robotY);
			if (nav.loc.grid.dist(path.peekLast(), robotLoc) < 3){
				s.leftDriveMotor.setSpeed(0);
				s.rightDriveMotor.setSpeed(0);
				return 1;
			}
			else{
				pfs.step(nav,s);
				return 1;
			}
		}
	}
}
