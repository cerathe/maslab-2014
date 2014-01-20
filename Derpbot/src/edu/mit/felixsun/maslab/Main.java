package edu.mit.felixsun.maslab;

import java.awt.BorderLayout;
import java.awt.image.BufferedImage;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

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
	public SparseGrid grid;
	public double gridSize = 1;		// Inches / square
	public double robotWidth = 10;	// Inches
	public double[] wall;
	public BotClientMap map = BotClientMap.getDefaultMap();
	public Mat processedImage;
	public cvData() {
		offset = -2;
		map = BotClientMap.getDefaultMap();
		grid = new SparseGrid(gridSize, map);
		grid.writeMap();
		wall = new double[] {0,0,0,0,0};
	}
	public cvData(BotClientMap mp) {
		offset = -2;
		grid = new SparseGrid(gridSize, mp);
		wall = new double[] {0,0,0,0,0};
		map = mp;
		grid.writeMap();
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
		robotX = 5;
		robotY = 5;
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
	
	public double trueMeas(double viewTheta, double x, double y, double theta){
		//The true measurement at angle viewtheta given position x,y,theta.
		double absTheta = viewTheta + theta;
		double xinc = Math.cos(absTheta);
		double yinc = Math.sin(absTheta);
		double xtest = x;
		double ytest = y;
		double ans = -1;
		if(absTheta==Math.PI/2 || absTheta == 3*Math.PI/2){
			while(ytest<this.maxY && ytest>0){
				ytest = ytest + yinc;
				if(filled(xtest,ytest)){
					ans = dist(x,y,xtest,ytest);
					System.out.println(xtest);
					break;
				}
			}
		}
		else{
			while(xtest<this.maxX && xtest>0){
				xtest = xtest + xinc;
				ytest = ytest + yinc;
				if(filled(xtest,ytest)){
					ans = dist(x,y,xtest,ytest);
					System.out.println(get(xtest,ytest));
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
	public final int CAM_MODE = 1;
	public final boolean SHOW_IMAGES = true;
	// 0 = connected to robot
	// 1 = load image
	public cvData data = new cvData();
	Thread t;

	public void run(){
		String FILENAME = new String("/Users/vipul/git/maslab-2014/Derpbot/src/edu/mit/felixsun/maslab/corner3.jpg");
//		String FILENAME = new String("C:\\Users\\Felix\\Documents\\maslab\\walls.png");
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
	public static int SPEED = 20;
	public static double I_GAIN = 0;
    public static double WALLSPEED = 0.1;
    public static double I_GAIN_WALL = 0;
    public static double TOOCLOSE = 10;
    
	public static void main(String[] args) {
		// Just a testing framework for the computer vision stuff.
		cvHandle handle = new cvHandle(); // Run the cv stuff.
		Thread cvThread = new Thread(handle);
		cvThread.start();
		
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
