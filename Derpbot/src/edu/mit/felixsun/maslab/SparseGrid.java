package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentHashMap;

import comm.BotClientMap;
import comm.BotClientMap.Point;
import comm.BotClientMap.Pose;
import comm.BotClientMap.Wall;

class SparseGrid {
	/*
	 * A sparse-matrix style 2D grid, that stores an integer at each point.
	 */
	public double gridSize; // Inches per grid square
	public ConcurrentHashMap<Entry<Integer, Integer>, Integer> map;
	public BotClientMap theMap;
	public double BASELINE_PROB = 0.01;		// Probability that we observe a random (wrong) wall segment.
	public List<Integer> wallNumbers = Arrays.asList(1, 2, 3, 4);
	List<SimpleEntry<Integer,Integer>> voidArea = new ArrayList<SimpleEntry<Integer,Integer>>();
	double robotX;
	double robotY;
	double robotTheta;
	double maxX;
	double maxY;
	double width;
	int voidWidth; //clearance from the walls in grid spaces
	
	static double MEAS_SIGMA = 4; //Estimated st.dev of distance measurement.
	
	public SparseGrid(double scale, BotClientMap theMap, double robotWidth) {
		gridSize = scale;
		map = new ConcurrentHashMap<Entry<Integer, Integer>, Integer>();
		this.theMap = theMap;
		robotX = 20;
		robotY = 20;
		robotTheta = Math.PI/2;
		maxX = 0;
		maxY = 0;
		width = robotWidth;
		voidWidth = 4;
		this.writeMap();
	}
	
	public void writeMap(){
		double mX = 0;
		double mY = 0;
		for(int i = 0; i<theMap.walls.size(); i++){
			Wall thisWall = theMap.walls.get(i);
			//Walls are scaled in theMap.gridSize inches
			//our scale is in this.gridSize inches
			double scaleFactor = (theMap.gridSize/this.gridSize);
			int startx = (int) (thisWall.start.x * scaleFactor);
			int starty = (int) (thisWall.start.y * scaleFactor);
			int endx = (int) (thisWall.end.x * scaleFactor);
			int endy = (int) (thisWall.end.y * scaleFactor);
			if(Math.max(startx, endx)>mX){mX = Math.max(startx,endx);}
			if(Math.max(starty, endy)>mY){mY = Math.max(starty,endy);}
			double slope = (endy-starty)/((double)(endx - startx));
			//Put the wall on the grid
			if(endx==startx){
				for(int y = 0; Math.abs(y)<Math.abs(endy-starty); y+= endy<starty? -1: 1){
					set(endx,y+starty, thisWall.type.ordinal());
				}
			}
			for(int x = 0; Math.abs(x)<Math.abs(endx-startx); x+= endx<startx? -1: 1){
				set(startx + x,(int)(starty + slope*x), thisWall.type.ordinal());
			}
		}
		this.maxX = mX;
		this.maxY = mY;
	}
	
	public void set(double x, double y, int value) {
		int xIndex = (int) (x / gridSize);
		int yIndex = (int) (y / gridSize);
		SimpleEntry<Integer, Integer> coords = new SimpleEntry<Integer, Integer>(xIndex, yIndex);
		for(int i = -voidWidth; i<voidWidth+1; i++){
			for(int j = -voidWidth; j<voidWidth+1; j++){
				voidArea.add(new SimpleEntry<Integer,Integer>(xIndex+i, yIndex+j));
			}
		}
		map.put(coords, value);
	}
	
	/*
	 * Important: safeSet does not add to the void area. 
	 * set creates a void area of voidWidth around any points added.
	 */
	public void safeSet(double x, double y, int value) {
		int xIndex = (int) (x / gridSize);
		int yIndex = (int) (y / gridSize);
		SimpleEntry<Integer, Integer> coords = new SimpleEntry<Integer, Integer>(xIndex, yIndex);
		map.put(coords, value);
	}
	
	public int get(double x, double y) {
		int xIndex = (int) (x / gridSize);
		int yIndex = (int) (y / gridSize);
		SimpleEntry<Integer, Integer> coords = new SimpleEntry<Integer, Integer>(xIndex, yIndex);
		if (map.containsKey(coords)){
			return map.get(coords);
		} else {
			return -1;
		}
	}
	
	public boolean filled(double x, double y) {
		int xIndex = (int) (x / gridSize);
		int yIndex = (int) (y / gridSize);
		SimpleEntry<Integer, Integer> coords = new SimpleEntry<Integer, Integer>(xIndex, yIndex);
		return map.containsKey(coords);
	}
	
	public double dist(double x1, double y1, double x2, double y2){
		return Math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
	}
	
	public ArrayList<Point> generateRing(double centerX, double centerY, double radius) {
		ArrayList<Point> out = new ArrayList<Point>();
		double x = centerX - radius;
		double y = centerY - radius;
		// Go left
		while (x < centerX + radius) {
			out.add(new Point(x, y));
			x += gridSize;
		}
		// Go up
		while (y < centerY + radius) {
			out.add(new Point(x, y));
			y += gridSize;
		}
		// Go right
		while (x > centerX - radius) {
			out.add(new Point(x, y));
			x -= gridSize;
		}
		// Go down
		while (y > centerY + radius) {
			out.add(new Point(x, y));
			y -= gridSize;
		}
		return out;
	}
	
	public Point closestOccupied(double x, double y, double maxR) {
		/*
		 * Finds the closest occupied point to (x, y) in the map.
		 * Returns (-1, -1) if no point is found within maxR.
		 */
		if (filled(x, y)) {
			return new Point(x, y);
		}
		Point bestSoFar = new Point(-1, -1);
		double bestDistance = 1000;
		for(double r = gridSize; r < maxR; r += gridSize){
			ArrayList<Point> toCheck = generateRing(x, y, r);
			for (Point thisP : toCheck) {
				if (filled(thisP.x, thisP.y)) {
					double thisDist = dist(x, y, thisP.x, thisP.y);
					if (thisDist < bestDistance) {
						bestSoFar = thisP;
						bestDistance = thisDist;
					}
				}
			}
			if (bestDistance < r) {
				return bestSoFar;
			}
		}
		return bestSoFar;
		
	}
	
	public double trueMeas(double viewTheta, double x, double y, double theta){
		//The true measurement at angle viewtheta given position x,y,theta.
		double absTheta = viewTheta + theta;
		double xinc = Math.cos(absTheta) * gridSize;
		double yinc = Math.sin(absTheta) * gridSize;
		double xtest = x;
		double ytest = y;
		double ans = -1;
		if(absTheta==Math.PI/2 || absTheta == 3*Math.PI/2){
			// Felix doesn't think this branch will ever get run, but he's leaving it here for now.
			while(ytest<this.maxY && ytest>0){
				ytest = ytest + yinc;
				if(filled(xtest,ytest)){
					ans = dist(x,y,xtest,ytest);
//					System.out.println(xtest);
					break;
				}
			}
		}
		else{
			// If this is slow, add an extra termination condition: a max distance.
			while(xtest<this.maxX && xtest>0){
				xtest = xtest + xinc;
				ytest = ytest + yinc;
				if(filled(xtest,ytest)){
					ans = dist(x,y,xtest,ytest);
//					System.out.println(get(xtest,ytest));
					break;
				}
			}
		}
		return ans;
	}
	
	public double[] expMeas(Pose pose, double[] measurements){
		//Expected measurement given the pose (and the real measurements for size)
		int size = measurements.length;
		double[] output = new double[size];
		for(int i=0; i<size; i++){
			double theta = ImageProcessor.angularPosition(i, size) - Math.PI/2;
			double reality = trueMeas(theta,pose.x,pose.y,pose.theta);
			output[i] = reality;
		}
		return output;
	}
	
	private double gaussian(double x, double s){
		//normal distribution
		return Math.exp(-Math.pow(x, 2)/(2*Math.pow(s,2)))/(s*Math.sqrt(2*Math.PI));
	}
	
	public double noisyMeasurement(double viewTheta, double distance, double x, double y, double theta){
		//Gives a probability of measuring a certain distance at angle viewtheta given the position (x,y,theta).
		double absTheta = viewTheta + theta - Math.PI/2;
		double targetX = x + width/2 * Math.cos(theta) + distance * Math.cos(absTheta);
		double targetY = y + width/2 * Math.sin(theta) + distance * Math.sin(absTheta);
		Point closest = closestOccupied(targetX, targetY, 12);
		double diff = dist(targetX, targetY, closest.x, closest.y);
		return Math.log(Math.max(gaussian(diff, MEAS_SIGMA), BASELINE_PROB));
	}
	
	public double stateLogProb(HashMap<Double, Double> measurements, double x, double y, double theta) {
		/*
		 * Returns the log probability of measurements being observed, given robot position (x, y, theta)
		 * Currently, only accounts for walls.
		 * TODO:
		 * - Account for field landmarks.
		 * - Account for forward-facing ultrasound?
		 */
		double logProb = 0;
		for (Map.Entry<Double, Double> entry : measurements.entrySet()) {
			logProb += noisyMeasurement(entry.getKey(), entry.getValue(), x, y, theta); // / entry.getValue();
		}
		return logProb;
	}

	public double measLikelihood(Pose pose, double[] measurements){
		/*
		 * Measuring likelihood of position x given measurement m: We have:
		 * p(m|x) = p(x|m)*p(m)/p(x)
		 * Make naive assumption that each m is equally likely
		 * and each candidate position x is equally likely 
		 * 		^ may want to change the prior to be weighted in direction of motion
		 * 		but this is complicated.
		 * then p(m|x) \propto p(x|m). So comparing p(m|x) will give us which positions
		 * are more likely
		 */
		double[] reality = expMeas(pose, measurements);
		double output=1;
		double diff;
		for(int i=0; i<reality.length; i++){
			diff = Math.abs(measurements[i]-reality[i]);
			output = output * (gaussian(diff,MEAS_SIGMA));
		}
		
		return output;
		
	}
	
	
	public void removeIslands() {
		/*
		 * Clears any isolated wall blocks.
		 * (Wall blocks with nothing in the 8 positions around them.)
		 * Probably unnecessary now.
		 */
		for (Entry<Integer, Integer> pair : map.keySet()) {
			int x = pair.getKey();
			int y = pair.getValue();
			boolean saveThis = false;
			for (int deltaX = -1; deltaX < 2; deltaX++) {
				for (int deltaY = -1; deltaY < 2; deltaY++) {
					if (deltaX == 0 && deltaY == 0) {
						continue;
					}
					if (wallNumbers.contains(
							map.get(new SimpleEntry<Integer, Integer>(x+deltaX, y+deltaY))
						)){
						saveThis = true;
						break;
					}
				}
				if (saveThis) {
					break;
				}
			}
			if (!saveThis) {
				map.remove(pair);
			}
		}
	}
	public boolean allowedSpace(Entry<Integer, Integer> pt){
		boolean allowed = true;
		for(int i=0; i<voidArea.size(); i++){
			if(voidArea.get(i).equals(pt)){
				allowed = false;
			}
		}
		return allowed;
	}
	
	public boolean allowedSpace(int x, int y){
		Entry<Integer, Integer>pt = new SimpleEntry<Integer, Integer>(x,y);
		boolean allowed = true;
		for(int i=0; i<voidArea.size(); i++){
			if(voidArea.get(i).equals(pt)){
				allowed = false;
			}
		}
		return allowed;
	}
	
	public List<SimpleEntry<Integer, Integer>> getNeighbors(SimpleEntry<Integer, Integer> pt){
		List<SimpleEntry<Integer, Integer>> output = new ArrayList<SimpleEntry<Integer, Integer>>();
		int x = pt.getKey();
		int y = pt.getValue();
		for(int i=-1; i<2; i++){
			for(int j=-1; j<2; j++){
				if(i!=0 && j!=0){
					if(allowedSpace(x+i,y+j)){
						output.add(new SimpleEntry<Integer, Integer>(x+i, y+j));
					}
				}
			}
		}
		return output;
	}

}