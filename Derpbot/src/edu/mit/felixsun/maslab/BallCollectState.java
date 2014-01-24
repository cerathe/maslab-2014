package edu.mit.felixsun.maslab;

import java.util.LinkedList;
import java.util.AbstractMap.SimpleEntry;
import java.util.Random;

public class BallCollectState extends State {
	
	Random rng = new Random();
	PathFollowState myPath;
	
	public BallCollectState(Navigation nav) {
		newGoal(nav);
	}
	
	void newGoal(Navigation nav) {
		SimpleEntry<Integer,Integer> iPos = new SimpleEntry<Integer,Integer>((int) nav.loc.grid.robotX,(int) nav.loc.grid.robotY);
		LinkedList<SimpleEntry<Integer, Integer>> naiveWay = new LinkedList<SimpleEntry<Integer, Integer>>();
		while (naiveWay.size() == 0) {
			int destX = rng.nextInt((int) nav.loc.grid.maxX);
			int destY = rng.nextInt((int) nav.loc.grid.maxY);
			SimpleEntry<Integer,Integer> destination = new SimpleEntry<Integer,Integer>(destX, destY);
			naiveWay = nav.naiveWallFollow(iPos, destination);
		}
		LinkedList<SimpleEntry<Integer, Integer>> theWay = nav.cleanUpNaive(naiveWay);
		myPath = new PathFollowState(0.1, theWay);
	}
	
	public void step(Navigation nav, Sensors sensors){
		// 1 - If we see a ball, go for it.
		if (nav.loc.landmarks.size() > 0) {
			
		}
		
		// 2 - If we were chasing a ball, but don't see it anymore, plow forward a little
		// to make sure we pick it up.
		
		// 3 - Otherwise, keep nav-ing to random points.
	}
}
