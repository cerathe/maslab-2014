package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;

public class Navigation {
	public Localization loc;
	public final int MAX_TRIES = 100;
	public Navigation(Localization loca){
		loc = loca;
	}

	public LinkedList<SimpleEntry<Integer, Integer>> straightLine(int x1, int y1, int x2, int y2){
		//returns the start and endpoints of a straight line in the right direction
		//up to the first obstruction.
		double xint;
		double yint;
		if (x2 == x1){
			xint = 0;
			yint = y2>y1? 1:-1;
		}
		else if(y2==y1){
			xint = x2>x1? 1:-1;
			yint = 0;
		}
		else{
			double slope = Math.abs((double)(y2-y1))/((double)(x2-x1));
			if(slope<1){
				xint = x2>x1? 1: -1;
				yint = y2>y1? slope:-slope;
			}
			else{
				xint = x2>x1? 1/slope: -1/slope;
				yint = y2>y1? 1:-1;
			}
		}
		LinkedList<SimpleEntry<Integer,Integer>> path = new LinkedList<SimpleEntry<Integer,Integer>>();
		path.add(new SimpleEntry<Integer,Integer>(x1,y1));
		boolean term = false;
		double x = x1;
		double y = y1;
		while(!term){
			x = x+xint;
			y = y+yint;
			if((int)x == x2){
				path.add(new SimpleEntry<Integer,Integer>((int)x, (int)y));
				term = true;
			}
			else if(!loc.grid.allowedSpace(new SimpleEntry<Integer,Integer>((int)x, (int)y))){
				term=true;
			}
			else{
				path.add(new SimpleEntry<Integer,Integer>((int)x, (int)y));
			}
		}
		return path;
	}

	public LinkedList<SimpleEntry<Integer, Integer>> straightLine(SimpleEntry<Integer, Integer> p1, SimpleEntry<Integer, Integer> p2){
		//returns the start and endpoints of a straight line in the right direction
		//up to the first obstruction.
		int x1 = p1.getKey();
		int x2 = p2.getKey();
		int y1 = p1.getValue();
		int y2 = p2.getValue();
		return straightLine(x1,y1,x2,y2);
	}

	public LinkedList<SimpleEntry<Integer, Integer>> naiveWallFollow(int x1, int y1, int x2, int y2){
		//Naive wall following: 
		LinkedList<SimpleEntry<Integer,Integer>> path = new LinkedList<SimpleEntry<Integer,Integer>>();
		SimpleEntry<Integer,Integer> p1 = new SimpleEntry<Integer, Integer>(x1,y1);
		SimpleEntry<Integer,Integer> p2 = new SimpleEntry<Integer, Integer>(x2,y2);
		//first Try
		path.add(p1);
		path.add(straightLine(p1,p2).getLast());
		HashSet<SimpleEntry<Integer,Integer>> traversedClosePts = new HashSet<SimpleEntry<Integer,Integer>>(); //points touching the void that have been checked.
		LinkedList<SimpleEntry<Integer,Integer>> nextPts = new LinkedList<SimpleEntry<Integer,Integer>>(); //points to check
		nextPts.addAll(loc.grid.getNeighbors(path.getLast()));
		
		//traverse the wall until you hit a corner.
		
		//Places that count as hitting the target.
		List<SimpleEntry<Integer, Integer>> finalSpots = loc.grid.getNeighbors(p2);
		int tries = 0;
		
		//Assumes a path exists. which is fair.
		while(!path.getLast().equals(p2) && tries < MAX_TRIES){
			tries ++;
			//pick out the next point to check.
			SimpleEntry<Integer,Integer> thisPt = nextPts.poll();
			
			//If we've already been here before, don't bother.
			if(traversedClosePts.contains(thisPt)){
				continue;
			}
			
			//Get the neighbors of thisPt that are touching a wall
			LinkedList<SimpleEntry<Integer, Integer>> neighbors = loc.grid.getWallNeighbors(thisPt);
			
			//Try to draw a straight line to the target.
			LinkedList<SimpleEntry<Integer, Integer>> bestGuess = straightLine(thisPt, p2);
			//how far you got before hitting something.
			SimpleEntry<Integer, Integer> attempt = bestGuess.getLast();
			//if you hit the target, stop.
			if(finalSpots.contains(attempt)){
				path.add(thisPt);
				path.add(attempt);
				break;
			}
			else{
				//If you have only one or 3 wall neighbors, you've hit a corner
				if(neighbors.size()==1 || neighbors.size()==3);
					//Add the corner to the path.
					path.add(thisPt);
					//refresh the list of new points to try, .
					nextPts.clear();
					nextPts.addAll(neighbors);
				}
			//either way, you've been here.
			traversedClosePts.add(thisPt);
		}
		if (tries >= MAX_TRIES) {
			// Turns out, we didn't actually find a path...
			return new LinkedList<SimpleEntry<Integer,Integer>>();
		}
		return path;
		/*
		 * I think this is faster than A* because the search
		 * space is smaller (only wall-like things) and there's no heuristic to compute
		 * for each point.
		 * 
		 * This also assumes that all the walls are straight so it only works for
		 * things like this.
		 */
	}
	public LinkedList<SimpleEntry<Integer, Integer>> naiveWallFollow(SimpleEntry<Integer, Integer>pt1, SimpleEntry<Integer, Integer>pt2){
		int x1 = pt1.getKey();
		int x2 = pt2.getKey();
		int y1 = pt1.getValue();
		int y2 = pt2.getValue();
		return naiveWallFollow(x1,y1,x2,y2);
	}

	public LinkedList<SimpleEntry<Integer, Integer>> cleanUpNaive(LinkedList<SimpleEntry<Integer,Integer>> naive){
		//This cleanup runs in O(n^2)--is there better?
		LinkedList<SimpleEntry<Integer, Integer>> finalPath = new LinkedList<SimpleEntry<Integer,Integer>>();
		
		Iterator<SimpleEntry<Integer, Integer>> it = naive.iterator();
		finalPath.add(naive.getFirst());
		
		SimpleEntry<Integer,Integer> next;
		SimpleEntry<Integer,Integer> secondNext;
		next = it.next();
		while(it.hasNext()){
			Iterator<SimpleEntry<Integer, Integer>> it2 = naive.descendingIterator();
			secondNext = it2.next();
			SimpleEntry<Integer, Integer> triedLine = straightLine(next,secondNext).getLast();
			while(!secondNext.equals(next)){
				if(triedLine.equals(secondNext)){
					break;
				}
				secondNext = it2.next();
				triedLine = straightLine(next,secondNext).getLast();
			}
			finalPath.add(secondNext);
			if(!secondNext.equals(next)){
				while(!secondNext.equals(next)){
					next = it.next();
				}				
			}
			else{
				next = it.next();
			}
		}
		return finalPath;
	}

	public void drawPath(LinkedList<SimpleEntry<Integer, Integer>> x){	
		Iterator<SimpleEntry<Integer,Integer>> it = x.iterator();
		SimpleEntry<Integer,Integer> thisOne = it.next();
		while(it.hasNext()){
			SimpleEntry<Integer,Integer> nextOne = it.next();
			loc.grid.drawList(straightLine(thisOne, nextOne));
			thisOne = nextOne;
		}
	}

}
