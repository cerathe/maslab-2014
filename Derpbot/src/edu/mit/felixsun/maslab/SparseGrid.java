package edu.mit.felixsun.maslab;

import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import comm.BotClientMap;
import comm.BotClientMap.Point;
import comm.BotClientMap.Pose;
import comm.BotClientMap.Wall;

public class SparseGrid {
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
	Set<SimpleEntry<Integer,Integer>> voidArea = new HashSet<SimpleEntry<Integer,Integer>>();
	public double robotX;
	public double robotY;
	public double robotTheta;
	double maxX;
	double maxY;
	double minX;
	double minY;
	double width;
	int voidWidth; //clearance from the walls in grid spaces
	int reactorVoidWidth; //clearance from the reactors in grid spaces
	List<PointOfInterest> places= new LinkedList<PointOfInterest>(); 
	
	public class PointOfInterest{
		public boolean visited = false;
		public int ballsIn = 0;
		public SimpleEntry<Integer,Integer> actualMidpoint;
		public SimpleEntry<Integer,Integer> normPt1;
		public SimpleEntry<Integer,Integer> normPt2;
		public int type;
		
		public PointOfInterest(int typ, SimpleEntry<Integer,Integer> midpt, SimpleEntry<Integer,Integer> n1, SimpleEntry<Integer,Integer> n2){
			actualMidpoint = midpt;
			normPt1 = n1;
			normPt2 = n2;
			type = typ;
		}
		
		public void score(){
			ballsIn++;
		}
		
		public void visit(){
			visited = true;
		}
		
	}

	public List<List<Entry<Double, Double>>> landmarks;
	static double MEAS_SIGMA = 1; //Estimated st.dev of distance measurement.
	static double LANDMARK_SIGMA = 0.1;
	
	public SparseGrid(double scale, BotClientMap theMap) {
		gridSize = scale;
		map = new ConcurrentHashMap<Entry<Integer, Integer>, Integer>();
		errorDistances = new ConcurrentHashMap<Entry<Integer, Integer>, Integer>();
		this.theMap = theMap;
		maxX = 0;
		maxY = 0;
		minX = 10000;
		minY = 10000;
		width = Constants.ROBOT_WIDTH;
		voidWidth = (int) (width / 2 + 1);
		reactorVoidWidth = voidWidth + 6;
		landmarks = new ArrayList<List<Entry<Double, Double>>>();
		for (int i=0; i<3; i++) {
			landmarks.add(new ArrayList<Entry<Double, Double>>());
		}
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
			if(Math.max(startx, endx)>maxX){maxX = Math.max(startx,endx);}
			if(Math.max(starty, endy)>maxY){maxY = Math.max(starty,endy);}
			if(Math.min(startx, endx)<minX){minX = Math.min(startx, endx);}
			if(Math.min(starty, endy)<minY){minY = Math.min(startx, endx);}
			
			double slope = (endy-starty)/((double)(endx - startx));
			// If this wall is a landmark, mark it.
			if (thisWall.type.ordinal() == 3) {
				// Reactor.
				Entry<Double, Double> pt = new SimpleEntry<Double, Double> (
						(startx+endx)/2.0, (starty+endy)/2.0);
				landmarks.get(0).add(pt);
			} else if (thisWall.type.ordinal() == 1) {
				// Opponent wall.
				Entry<Double, Double> pt = new SimpleEntry<Double, Double> (
						(startx+endx)/2.0, (starty+endy)/2.0);
				landmarks.get(1).add(pt);
			}
			
			//Put the wall on the grid
			if(endx==startx){
				for(int y = 0; Math.abs(y)<Math.abs(endy-starty); y+= endy<starty? -1: 1){
					set(endx,y+starty, thisWall.type.ordinal());
				}
			}
			for(int x = 0; Math.abs(x)<Math.abs(endx-startx); x+= endx<startx? -1: 1){
				set(startx + x,(int)(starty + slope*x), thisWall.type.ordinal());
			}
			
			//Process the points of interest
			if(thisWall.type.ordinal()>0){
				double midx = (startx+endx)/2;
				double midy = (starty+endy)/2;
				SimpleEntry<Integer,Integer> midPt = new SimpleEntry<Integer,Integer>((int) midx, (int)midy);
				SimpleEntry<Integer,Integer> normPt1;
				SimpleEntry<Integer,Integer> normPt2;
				if(endx == startx){
					normPt1 = new SimpleEntry<Integer,Integer>((int)(startx + reactorVoidWidth+5), (int) midy);
					normPt2 = new SimpleEntry<Integer,Integer>((int)(startx - reactorVoidWidth-5), (int) midy);
				}
				else if(endy == starty){
					normPt1 = new SimpleEntry<Integer,Integer>((int)(midx), (int) midy + reactorVoidWidth+5);
					normPt2 = new SimpleEntry<Integer,Integer>((int)(midx), (int) midy - reactorVoidWidth-5);
				}
				else{
					double normSlp = -1/slope;
					normPt1 = new SimpleEntry<Integer,Integer>((int)(midx + reactorVoidWidth+ 5), (int)(midy + normSlp*(reactorVoidWidth+5)));
					normPt2 = new SimpleEntry<Integer,Integer>((int)(midx - reactorVoidWidth- 5), (int)(midy - normSlp*(reactorVoidWidth+5)));
				}
				
				
				PointOfInterest aNewPlace = new PointOfInterest(thisWall.type.ordinal(), midPt, normPt1, normPt2);
				places.add(aNewPlace);
			}
			
		}
		
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
		if(value==3){
			for(int i = -reactorVoidWidth; i<reactorVoidWidth + 1; i++){
				for(int j = -reactorVoidWidth; j<reactorVoidWidth+1; j++){
					voidArea.add(new SimpleEntry<Integer,Integer>(xIndex+i, yIndex+j));
				}
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
	
	public double dist(SimpleEntry<Integer,Integer> p1,SimpleEntry<Integer,Integer> p2 ){
		return dist(p1.getKey(), p1.getValue(), p2.getKey(), p2.getValue());
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
	
	public double trueMeas(double viewTheta, double x, double y, double theta, double max_d){
		//The true measurement at angle viewtheta given position x,y,theta.
		double absTheta = viewTheta + theta - Math.PI/2;
		double xinc = Math.cos(absTheta) * gridSize / 2;
		double yinc = Math.sin(absTheta) * gridSize / 2;
		double xtest = x;
		double ytest = y;
		int increments = 0;
		double ans = 2 * max_d / gridSize;
		// If this is slow, add an extra termination condition: a max distance.
		while(increments < max_d / gridSize){
			increments++;
			xtest = xtest + xinc;
			ytest = ytest + yinc;
			if(filled(xtest,ytest)){
				ans = dist(x,y,xtest,ytest);
//					System.out.println(get(xtest,ytest));
				break;
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
			double reality = trueMeas(theta,pose.x,pose.y,pose.theta, 48);
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
	
	public double stateLogProb(cvData data, double x, double y, double theta) {
		/*
		 * Returns the log probability of measurements being observed, given robot position (x, y, theta)
		 * Currently, only accounts for walls.
		 * TODO:
		 * - Account for field landmarks. - DONE.
		 * - Account for forward-facing ultrasound? - NOT USING.
		 */
		double logProb = 0;
		for (Map.Entry<Double, Double> entry : data.angles.entrySet()) {
			logProb += noisyMeasurement(entry.getKey(), entry.getValue(), x, y, theta); // / entry.getValue();
		}
		// Now, account for landmarks.
		for (int i=0; i<data.landmarks.size(); i++) {
			if (data.landmarks.get(i) == null) {
				continue;
			}
			for (Entry<Double, Double> landmark: data.landmarks.get(i)) {
				// First, convert to field coordinates.
				double absTheta = landmark.getValue() + theta - Math.PI/2;
				double targetX = x + width/2 * Math.cos(theta) + landmark.getKey() * Math.cos(absTheta);
				double targetY = y + width/2 * Math.sin(theta) + landmark.getKey() * Math.sin(absTheta);
				double closestD = findLandmarkError(targetX, targetY, i);
//				logProb += Math.log(gaussian(closestD, LANDMARK_SIGMA));
			}
		}
		return logProb;
	}
	
	public double findLandmarkError(double x, double y, int type) {
		/*
		 * Finds the distance to the closest landmark of type, from location.
		 */
		if (landmarks.get(type) == null || landmarks.get(type).size() == 0) {
			return 0;
		}
		double bestD = 1000000;
		for (Entry<Double, Double> candidate : landmarks.get(type)) {
			double thisDist = dist(candidate.getKey(), candidate.getValue(), x, y);
			if (thisDist < bestD) {
				bestD = thisDist;
			}
		}
		return bestD;
		
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
		if(voidArea.contains(pt)){
			allowed = false;
		}
		return allowed;
	}
	
	public boolean allowedSpace(int x, int y){
		Entry<Integer, Integer>pt = new SimpleEntry<Integer, Integer>(x,y);
		boolean allowed = true;
		if(voidArea.contains(pt)){
			allowed = false;
		}
		return allowed;
	}
	
	public LinkedList<SimpleEntry<Integer, Integer>> getNeighbors(SimpleEntry<Integer, Integer> pt){
		LinkedList<SimpleEntry<Integer, Integer>> output = new LinkedList<SimpleEntry<Integer, Integer>>();
		int x = pt.getKey();
		int y = pt.getValue();
		for(int i=-1; i<2; i++){
			for(int j=-1; j<2; j++){
				if(!(i==0 && j==0)){
					if(allowedSpace(x+i,y+j)){
						output.add(new SimpleEntry<Integer, Integer>(x+i, y+j));
					}
				}
			}
		}
		return output;
	}
	
	public HashSet<SimpleEntry<Integer, Integer>> getNeighborsSet(SimpleEntry<Integer, Integer> pt){
		HashSet<SimpleEntry<Integer, Integer>> output = new HashSet<SimpleEntry<Integer, Integer>>();
		int x = pt.getKey();
		int y = pt.getValue();
		for(int i=-1; i<2; i++){
			for(int j=-1; j<2; j++){
				if(allowedSpace(x+i,y+j)){
					output.add(new SimpleEntry<Integer, Integer>(x+i, y+j));
				}
			}
		}
		return output;
	}
	
	public LinkedList<SimpleEntry<Integer, Integer>> getWallNeighbors(SimpleEntry<Integer, Integer> pt){
		LinkedList<SimpleEntry<Integer, Integer>> output = getNeighbors(pt);
		LinkedList<SimpleEntry<Integer, Integer>> realoutput = new LinkedList<SimpleEntry<Integer, Integer>>();
		Iterator<SimpleEntry<Integer, Integer>> it = output.iterator();
		while(it.hasNext()){
			SimpleEntry<Integer, Integer> x = it.next();
			if(getNeighbors(x).size()<8){
				realoutput.add(x);
			}
		}
		return realoutput;
	}
	
	public void drawList(LinkedList<SimpleEntry<Integer, Integer>> x){
		Iterator<SimpleEntry<Integer,Integer>> it = x.iterator();
		while(it.hasNext()){
			SimpleEntry<Integer,Integer> thisOne = it.next();
			safeSet(thisOne.getKey(), thisOne.getValue(), 3);
		}
	}


}