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
	
	public BallCollectState(Navigation nav) {
		newGoal(nav);
	}
	
	void newGoal(Navigation nav) {
		SimpleEntry<Integer,Integer> iPos = new SimpleEntry<Integer,Integer>((int) nav.loc.grid.robotX,(int) nav.loc.grid.robotY);
		LinkedList<SimpleEntry<Integer, Integer>> naiveWay = new LinkedList<SimpleEntry<Integer, Integer>>();
		while (naiveWay.size() == 0) {
			System.out.println("Pathing");
			int destX = rng.nextInt((int) nav.loc.grid.maxX);
			int destY = rng.nextInt((int) nav.loc.grid.maxY);
			SimpleEntry<Integer,Integer> destination = new SimpleEntry<Integer,Integer>(destX, destY);
			naiveWay = nav.naiveWallFollow(iPos, destination);
		}
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
		 * 1 - following ball
		 * 0 - looking
		 */
		// 0 - If stuck, back up for a bit, then choose a new goal.
		if (nav.loc.stuck) {
			stuckCount = 60;
		}
		
		if (stuckCount > 20) {
			// Stuck - back up.
			backState.step(nav.loc, sensors, -10);
			stuckCount--;
			return 0;
		}
		
		if (stuckCount > 0) {
			// Then turn for a bit.
			turnState.step(nav.loc, sensors, 1);
			stuckCount--;
			// At the very end, re-nav.
			if (stuckCount == 0) {
				newGoal(nav);
			}
			return 0;
		}
		
		// 1 - If we see a ball, go for it.
		int result = myBall.step(nav.loc, sensors);
		if (result == 1) {
			System.out.println("Ball");
			lastReturn = 1;
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
		lastReturn = 0;
		return 0;
	}
}
