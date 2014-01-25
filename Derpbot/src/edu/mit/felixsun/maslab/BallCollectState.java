package edu.mit.felixsun.maslab;

import java.util.LinkedList;
import java.util.AbstractMap.SimpleEntry;
import java.util.Random;

public class BallCollectState extends State {
	
	Random rng = new Random();
	PathFollowState myPath;
	BallFollowState myBall;
	int lastReturn = 0;
	
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
		myBall = new BallFollowState();
	}
	
	public int step(Navigation nav, Sensors sensors){
		/* 
		 * Return codes:
		 * 1 - following ball
		 * 0 - looking
		 */
		// 1 - If we see a ball, go for it.
		int result = myBall.step(nav.loc, sensors);
		if (result == 1) {
			lastReturn = 1;
			return 1;
		}
		// 2 - Otherwise, keep nav-ing to random points.
		if (lastReturn == 1) {
			newGoal(nav);
		}
		result = myPath.step(nav, sensors);
		if (result == 1) {
			newGoal(nav);
		}
		lastReturn = 0;
		return 0;
	}
}
