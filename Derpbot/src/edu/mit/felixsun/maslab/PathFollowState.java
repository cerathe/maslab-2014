package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;
import java.util.Iterator;
import java.util.LinkedList;

public class PathFollowState {
	double tolerance = 2;
	
	double driveSpeed;
	Iterator<SimpleEntry<Integer,Integer>> iterator;
	SimpleEntry<Integer,Integer> thisPt;
	PointTrackState pointTrack;
	LinkedList<SimpleEntry<Integer,Integer>> path;
	
	public PathFollowState(double speed, LinkedList<SimpleEntry<Integer,Integer>> path){
		driveSpeed = speed;
		iterator = path.iterator();
		thisPt = iterator.next();
		this.path = path;
		pointTrack  = new PointTrackState(driveSpeed);
	}
	
	public int step(Navigation nav, Sensors sensors){
		/*
		 * Return codes:
		 * 1 - Done.
		 * 0 - Still following.
		 */
//		System.out.println(path);
		SimpleEntry<Integer,Integer> currentPos = new SimpleEntry<Integer,Integer>((int)nav.loc.grid.robotX, (int)nav.loc.grid.robotY);
		double dist = nav.loc.grid.dist(currentPos, thisPt);
		if(dist<=tolerance){
			if(iterator.hasNext()){
				thisPt = iterator.next();
			}
			else{
				System.out.println("DONE");
				return 1;
			}
		}
		else{
			pointTrack.step(nav, sensors, thisPt);
		}
		return 0;
	}

}
