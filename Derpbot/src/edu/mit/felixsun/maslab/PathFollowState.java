package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;
import java.util.Iterator;
import java.util.LinkedList;

public class PathFollowState {
	double tolerance = 2;
	
	double driveSpeed;
	Iterator<SimpleEntry<Integer,Integer>> iterator;
	SimpleEntry<Integer,Integer> thisPt;
	PointTrackState pointTrack = new PointTrackState(driveSpeed);
	public PathFollowState(double speed, LinkedList<SimpleEntry<Integer,Integer>> path){
		driveSpeed = speed;
		iterator = path.iterator();
		thisPt = iterator.next();
	}
	
	public void step(Navigation nav, Sensors sensors){
		SimpleEntry<Integer,Integer> currentPos = new SimpleEntry<Integer,Integer>((int)nav.loc.grid.robotX, (int)nav.loc.grid.robotY);
		double dist = nav.loc.grid.dist(currentPos, thisPt);
		if(dist<=tolerance){
			if(iterator.hasNext()){
				thisPt = iterator.next();
			}
			else{
			}
		}
		else{
			pointTrack.step(nav, sensors, thisPt);
		}
	}

}
