package edu.mit.felixsun.maslab;

import java.util.LinkedList;
import java.util.AbstractMap.SimpleEntry;
import java.util.Random;

public class BallCollectState extends State {
	
	Random rng = new Random();
	PathFollowState myPath;
	BallFollowState myBall;
	DriveStraightState backState;
	TurnState turnState;
	int lastReturn = 0;
	int stuckCount = 0;
	int ramCount = 0;
	
	public BallCollectState(Navigation nav) {
		newGoal(nav);
	}
	
	void newGoal(Navigation nav) {
		SimpleEntry<Integer,Integer> iPos = new SimpleEntry<Integer,Integer>((int) nav.loc.grid.robotX,(int) nav.loc.grid.robotY);
		LinkedList<SimpleEntry<Integer, Integer>> naiveWay = new LinkedList<SimpleEntry<Integer, Integer>>();
		int destX, destY;
		SimpleEntry<Integer,Integer> destination = null;
		do {
//			System.out.println("Making new path");
			destX = rng.nextInt((int) nav.loc.grid.maxX);
			destY = rng.nextInt((int) nav.loc.grid.maxY);
			destination = new SimpleEntry<Integer,Integer>(destX, destY);
		} while (!nav.loc.grid.accessibleArea.containsKey(destination)); 
		naiveWay = nav.naiveWallFollow(iPos, destination);
		LinkedList<SimpleEntry<Integer, Integer>> theWay = nav.cleanUpNaive(naiveWay);
		System.out.println(theWay);
		myPath = new PathFollowState(Constants.SPEED, theWay);
		myBall = new BallFollowState();
		backState = new DriveStraightState();
		turnState = new TurnState();
	}
	
	public int step(Navigation nav, Sensors sensors){
		/* 
		 * Return codes:
		 * 3 - stuck
		 * 1 - following ball
		 * 0 - looking
		 */
		// 0 - If stuck, back up for a bit, then choose a new goal.
		if (ramCount > 0) {
			backState.step(nav.loc, sensors, 18);
			ramCount--;
		}
		
		// 1 - If we see a ball, go for it.
		int result = myBall.step(nav.loc, sensors);
		if (result == 3) {
			// Stuck
			return 3;
		}
		if (result == 1) {
			System.out.println("Ball");
			lastReturn = 1;
			ramCount = 40;
			return 1;
		}
		// 2 - Otherwise, keep nav-ing to random points.
		if (lastReturn == 1) {
			newGoal(nav);
		}
		System.out.println("Wander");
		result = myPath.step(nav, sensors);
		if (result == 1) {
			newGoal(nav);
		}
		else if(result==2) {
			newGoal(nav);
			return 3;
		}
		lastReturn = 0;
		return 0;
	}
}
