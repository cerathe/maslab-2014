package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;

public class Navigation {
	public Localization loc;
	public Navigation(Localization loca){
		loc = loca;
	}
	
	public LinkedList<SimpleEntry<Integer, Integer>> straightLine(int x1, int y1, int x2, int y2){
		//returns the start and endpoints of a straight line in the right direction
		//up to the first obstruction.
		double slope = (double)(y2-y1)/(double)(x2-x1);
		System.out.println(slope);
		LinkedList<SimpleEntry<Integer,Integer>> path = new LinkedList<SimpleEntry<Integer,Integer>>();
		path.add(new SimpleEntry<Integer,Integer>(x1,y1));
		boolean term = false;
		SimpleEntry<Integer,Integer> nextPt;
		double x = x1;
		double y = y1;
		int s;
		while(!term){
			s = path.size()-1;
			x = x+1;
			y = y+slope;
			nextPt = new SimpleEntry<Integer,Integer>((int)x, (int)y);
			if((int)x == x2 && (int)y == y2){
				path.add(nextPt);
				term = true;
			}
			if(loc.grid.voidArea.contains(nextPt)){
				term=true;
			}
			path.add(nextPt);
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
	
	public void naiveWallFollow(int x1, int y1, int x2, int y2){
		//Naive wall following: 
		LinkedList<SimpleEntry<Integer,Integer>> path = new LinkedList<SimpleEntry<Integer,Integer>>();
		SimpleEntry<Integer,Integer> p1 = new SimpleEntry<Integer, Integer>(x1,y1);
		SimpleEntry<Integer,Integer> p2 = new SimpleEntry<Integer, Integer>(x2,y2);
		//first Try
		path.addAll(straightLine(p1, p2));
		HashSet<SimpleEntry<Integer,Integer>> traversedWall; //void points that have been checked.
		HashSet<SimpleEntry<Integer,Integer>> traversedClosePts; //points touching the void that have been checked.
		while(!path.getLast().equals(p2)){
			LinkedList<SimpleEntry<Integer,Integer>> nextPts = new LinkedList<SimpleEntry<Integer,Integer>>();
			nextPts.add(path.getLast());
			//traverse along the wall until you get around the obstacle.
			/*
			 * practically: 
			 */
			boolean done = false;
			while(!done){
				for(int i=0; i<nextPts.size(); i++){
					//find all of the wall neighbors
					SimpleEntry<Integer,Integer> thisPt = nextPts.poll();
					if(straightLine(thisPt, p2).getLast()!=thisPt){
						done = true;
						path.add(thisPt);
					}
					//TODO: Finish
				}
			}
		}
		

	}
	
}
