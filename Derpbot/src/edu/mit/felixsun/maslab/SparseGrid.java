package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
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
	public ConcurrentHashMap<Entry<Integer, Integer>, Integer> errorDistances;
	public BotClientMap theMap;
	public double BASELINE_PROB = 0.0001;		// Probability that we observe a random (wrong) wall segment.
	public double MAX_ERROR_RADIUS = 18;		// The farthest out we search, when looking for the closest wall.
	public List<Integer> wallNumbers = Arrays.asList(1, 2, 3, 4);
	List<SimpleEntry<Integer,Integer>> voidArea = new ArrayList<SimpleEntry<Integer,Integer>>();
	double robotX;
	double robotY;
	double robotTheta;
	double maxX;
	double maxY;
	double width;
	int voidWidth; //clearance from the walls in grid spaces
	
	static double MEAS_SIGMA = 1; //Estimated st.dev of distance measurement.
	
	public SparseGrid(double scale, BotClientMap theMap, double robotWidth) {
		gridSize = scale;
		map = new ConcurrentHashMap<Entry<Integer, Integer>, Integer>();
		errorDistances = new ConcurrentHashMap<Entry<Integer, Integer>, Integer>();
		this.theMap = theMap;
		maxX = 0;
		maxY = 0;
		width = robotWidth;
		voidWidth = 4;
		this.writeMap();
		this.preprocessErrorDistances();
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
		
		// Get robot starting position.
		robotX = theMap.startPose.x * theMap.gridSize;
		robotY = theMap.startPose.y * theMap.gridSize;
		robotTheta = theMap.startPose.theta;
	}
	
	void preprocessErrorDistances() {
		/*
		 * Make it easier to determine how far away we are from the closest grid square.
		 * For each non-occupied grid square, compute the distance to the closest grid square.
		 */
		List<Entry<Integer, Integer>> frontier = new ArrayList<Entry<Integer, Integer>>();
		List<Entry<Integer, Integer>> newFrontier = new ArrayList<Entry<Integer, Integer>>();
		int[] xDirections = {-1, 1, 0, 0};
		int[] yDirections = {0, 0, -1, 1};
		for (Entry<Integer, Integer> coords : map.keySet()) {
			errorDistances.put(coords, 0);
			frontier.add(coords);
		}
		for (int radius = 1; radius < MAX_ERROR_RADIUS / gridSize; radius++) {
			for (Entry<Integer, Integer> coords : frontier) {
				for (int i = 0; i < 4; i++) {
					Entry<Integer, Integer> testCoords = new SimpleEntry<Integer, Integer>(
							coords.getKey() + xDirections[i], coords.getValue() + yDirections[i]);
					if (! errorDistances.containsKey(testCoords)) {
						errorDistances.put(testCoords, radius);
						newFrontier.add(testCoords);
					}
				}
			}
			frontier = newFrontier;
			newFrontier = new ArrayList<Entry<Integer, Integer>>();
		}
		
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
	
	public double closestOccupied(double x, double y) {
		/*
		 * Finds the distance to the closest occupied point from (x, y).
		 */
		Entry<Integer, Integer> coords = new SimpleEntry<Integer, Integer>((int)(x/gridSize), (int)(y/gridSize));
		if (! errorDistances.containsKey(coords)) {
			return MAX_ERROR_RADIUS;
		}
		int gridDist = errorDistances.get(coords);
		return gridDist * gridSize;
		
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
		if (distance > 72) {
			return 0;
		}
		double absTheta = viewTheta + theta - Math.PI/2;
		double targetX = x + width/2 * Math.cos(theta) + distance * Math.cos(absTheta);
		double targetY = y + width/2 * Math.sin(theta) + distance * Math.sin(absTheta);
		double diff = closestOccupied(targetX, targetY);;
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