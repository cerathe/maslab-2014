package edu.mit.felixsun.maslab;

import java.awt.BorderLayout;
import java.awt.image.BufferedImage;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.Random;

import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.sensors.Ultrasonic;
import edu.mit.felixsun.maslab.ImageProcessor;
import edu.mit.felixsun.maslab.Mat2Image;
import edu.mit.felixsun.maslab.WallFollowState;
import edu.mit.felixsun.maslab.SonarReadState;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import comm.BotClientMap;
import comm.BotClientMap.Point;
import comm.BotClientMap.Pose;
import comm.BotClientMap.Wall;
import comm.MapleComm;
import comm.MapleIO;
import jssc.SerialPort;
import jssc.SerialPortException;

class cvData {
	/*
	 * A bunch of data that gets passed between the computer vision system
	 * and the rest of the code.
	 */
	public double offset;
	public double gridSize = 1;		// Inches / square
	public double robotWidth = 10;	// Inches
	public Mat processedImage;
	HashMap<Double, Double> angles;
	public cvData() {
		offset = -2;
		angles = new HashMap<Double, Double>();
	}
}

class Sensors {
	public Ultrasonic ultraLeft;
	public Ultrasonic ultraRight;
	public Cytron leftDriveMotor;
	public Cytron rightDriveMotor;
}

class SparseGrid {
	/*
	 * A sparse-matrix style 2D grid, that stores an integer at each point.
	 */
	public double gridSize; // Inches per grid square
	public ConcurrentHashMap<Entry<Integer, Integer>, Integer> map;
	public BotClientMap theMap;
	public double BASELINE_PROB = 0.01;		// Probability that we observe a random (wrong) wall segment.
	public List<Integer> wallNumbers = Arrays.asList(1, 2, 3, 4);
	double robotX;
	double robotY;
	double robotTheta;
	double maxX;
	double maxY;
	
	static double MEAS_SIGMA = 0.1; //Estimated st.dev of distance measurement.
	
	public SparseGrid(double scale, BotClientMap theMap) {
		gridSize = scale;
		map = new ConcurrentHashMap<Entry<Integer, Integer>, Integer>();
		this.theMap = theMap;
		robotX = 20;
		robotY = 20;
		robotTheta = Math.PI/2;
		maxX = 0;
		maxY = 0;
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
	
	private double gaussian(double x, double s){
		//normal distribution
		return Math.exp(-Math.pow(x, 2)/(2*Math.pow(s,2)))/(s*Math.sqrt(2*Math.PI));
	}
	
	public double noisyMeasurement(double viewTheta, double distance, double x, double y, double theta){
		//Gives a probability of measuring a certain distance at angle viewtheta given the position (x,y,theta).
		double absTheta = viewTheta + theta;
		double targetX = x + distance * Math.cos(absTheta);
		double targetY = y + distance * Math.sin(absTheta);
		Point closest = closestOccupied(targetX, targetY, 6);
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
			logProb += noisyMeasurement(entry.getKey(), entry.getValue(), x, y, theta);
		}
		return logProb;
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

}

class cvHandle implements Runnable {
	/*
	 * Starts the cv scripts.  Runs in a separate thread.
	 */
	public final int CAM_MODE = 0;
	public final boolean SHOW_IMAGES = true;
	// 0 = connected to robot
	// 1 = load image
	public cvData data = new cvData();
	Thread t;

	public void run(){
//		String FILENAME = new String("/Users/vipul/git/maslab-2014/Derpbot/src/edu/mit/felixsun/maslab/corner3.jpg");
		String FILENAME = new String("C:\\Users\\Felix\\Documents\\maslab\\walls.png");
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		VideoCapture camera = new VideoCapture();
		Mat rawImage;
		int width, height;
		
		if (CAM_MODE == 0){
			// Setup the camera
			camera.open(1);
			
			// Create GUI windows to display camera output and OpenCV output
			width = (int) (camera.get(Highgui.CV_CAP_PROP_FRAME_WIDTH));
			height = (int) (camera.get(Highgui.CV_CAP_PROP_FRAME_HEIGHT));
			rawImage = new Mat();
		} else if (CAM_MODE == 1) {
			rawImage = Highgui.imread(FILENAME); 
			width = rawImage.width();
			height = rawImage.height();
		}
		JLabel cameraPane;
		JLabel opencvPane;

		if (SHOW_IMAGES) {
			cameraPane = createWindow("Camera output", width, height);
			opencvPane = createWindow("OpenCV output", width, height);
	
		}
		
		// Main loop 
		Mat processedImage = new Mat();
		// Vision timing.  How many fps do we get?
		long startTime = System.nanoTime();
		int frames = 0;
		
		while (true) {
			
			if (CAM_MODE == 0) {
				// Wait until the camera has a new frame
				camera.grab();
				camera.retrieve(rawImage);
			} else if (CAM_MODE == 1) {
				rawImage = Highgui.imread(FILENAME); 
			}
			// Process the image however you like
			cvData tempData = ImageProcessor.process(rawImage);
			synchronized (data) {
				data = tempData;
			}
			processedImage = tempData.processedImage;
			
			
			// Update the GUI windows
			if (SHOW_IMAGES) {
				updateWindow(cameraPane, rawImage);
				updateWindow(opencvPane, processedImage);
			}
			// Manually garbage collect, because opencv has memory issues :(
			System.gc();
			frames++;
			// Fps-counting stuff.  Do frames every second.
			if (System.nanoTime() - startTime > 1000000000) {
				System.out.println(frames);
				startTime = System.nanoTime();
				frames = 0;
			}
			
		}
	}
	
	public static JLabel createWindow(String name, int width, int height) {    
        JFrame imageFrame = new JFrame(name);
        imageFrame.setSize(width, height);
        imageFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        JLabel imagePane = new JLabel();
        imagePane.setLayout(new BorderLayout());
        imageFrame.setContentPane(imagePane);
        
        imageFrame.setVisible(true);
        return imagePane;
    }
    
    public static void updateWindow(JLabel imagePane, Mat mat) {
    	int w = (int) (mat.size().width);
    	int h = (int) (mat.size().height);
    	if (imagePane.getWidth() != w || imagePane.getHeight() != h) {
    		imagePane.setSize(w, h);
    	}
    	BufferedImage bufferedImage = Mat2Image.getImage(mat);
    	imagePane.setIcon(new ImageIcon(bufferedImage));
    }
}


public class Main {
	public final static int PARTICLE_COUNT = 100; 	// How many samples of the world?
	public final static int PRUNED_COUNT = 50;		// How many samples do we keep at the end of each step?
	public final static double TRAVEL_STEP = 0.5;	// Inches / step
	public final static double TURN_STEP = 0.1;		// Radians / step
	
	public static void main(String[] args) {
		// Just a testing framework for the computer vision stuff.
		cvHandle handle = new cvHandle(); // Run the cv stuff.
		Thread cvThread = new Thread(handle);
		cvThread.start();
		
		cvData data = handle.data;
		BotClientMap map = BotClientMap.getDefaultMap();
		SparseGrid grid = new SparseGrid(data.gridSize, map);
		ArrayList<Pose> robotPositions = new ArrayList<Pose>();
		for (int i=0; i<PARTICLE_COUNT; i++) {
			robotPositions.add(map.startPose);
		}
		double normalization = 0;
		grid.writeMap();
		
		JLabel cameraPane = cvHandle.createWindow("Derp", 600, 600);
		Random rng = new Random();

		while (true) {
		data = handle.data;
		
		// Update each particle with the expected drift.
		// Right now, the drift is Gaussian, but this can be a lot better, with encoder data.
		// Update the probability of each particle.
		for (int i=0; i<PARTICLE_COUNT; i++) {
			Pose oldPose = robotPositions.get(i);
			double newX = oldPose.x + rng.nextGaussian()*TRAVEL_STEP;
			double newY = oldPose.y + rng.nextGaussian()*TRAVEL_STEP;
			double newTheta = oldPose.theta + rng.nextGaussian()*TURN_STEP;
			double newProb = oldPose.prob + grid.stateLogProb(data.angles, newX, newY, newTheta) - normalization;
			robotPositions.set(i, new Pose(newX, newY, newTheta, newProb));
		}
		
		// Delete the unlikely particles.  Clone the likely particles.
		class ProbCompare implements Comparator<Pose> {
			public int compare(Pose a, Pose b) {
				if (a.prob > b.prob) {
					return 1;
				} else if (a.prob < b.prob) {
					return -1;
				} else {
					return 0;
				}
			}
		}
		Collections.sort(robotPositions, new ProbCompare());
		for (int i=PRUNED_COUNT; i<PARTICLE_COUNT; i++) {
			robotPositions.set(i, robotPositions.get(i % PRUNED_COUNT));
		}
		Pose bestGuess = robotPositions.get(0);
		normalization = bestGuess.prob;
		grid.robotX = bestGuess.x;
		grid.robotY = bestGuess.y;
		grid.robotTheta = bestGuess.theta;
		
		
		if (data.processedImage != null) {
			Mat finalMap = ImageProcessor.drawGrid(data.processedImage.size(), data, grid);
			cvHandle.updateWindow(cameraPane, finalMap);
		}
		try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

		
//		State sonarState = new SonarReadState();
//		State wallFollowState = new WallFollowState(-1, -1);
//		JLabel cameraPane = cvHandle.createWindow("Derp", 600, 600);
//		
//		// Start serial communication.
//		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
//		Sensors sensors = new Sensors();
//		sensors.ultraRight = new Ultrasonic(30, 29);
//		sensors.ultraLeft = new Ultrasonic(32, 31);
//		sensors.rightDriveMotor = new Cytron(2, 1);
//		sensors.leftDriveMotor = new Cytron(7, 6);
//		DigitalOutput ground1 = new DigitalOutput(0);
//		DigitalOutput ground2 = new DigitalOutput(5);
//		
//		comm.registerDevice(sensors.ultraLeft);
//		comm.registerDevice(sensors.ultraRight);
//		comm.registerDevice(sensors.leftDriveMotor);
//		comm.registerDevice(sensors.rightDriveMotor);
//		comm.registerDevice(ground1);
//		comm.registerDevice(ground2);
//		comm.initialize();
//		
//		ground1.setValue(false);
//		ground2.setValue(false);
//		comm.transmit();
//		while (true) {
//			cvData data = handle.data;
//			comm.updateSensorData();
////			System.out.println(sensors.ultraLeft.getDistance()*45);
//			sonarState.step(data, sensors);
//			if (data.processedImage != null) {
//				Mat finalMap = ImageProcessor.drawGrid(data.processedImage.size(), data);
//				cvHandle.updateWindow(cameraPane, finalMap);
//			}
//			wallFollowState.step(data, sensors);
//			comm.transmit();
//			
//			try {
//				Thread.sleep(10);
//			} catch (InterruptedException e) {
//				e.printStackTrace();
//			}
//		}

	}

}
