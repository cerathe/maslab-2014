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
import edu.mit.felixsun.maslab.DisplayWindow;

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

class cvHandle implements Runnable {
	/*
	 * Starts the cv scripts.  Runs in a separate thread.
	 */
	public final int CAM_MODE = 0;
	public final boolean SHOW_IMAGES = false;
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
		DisplayWindow cameraPane;
		DisplayWindow opencvPane;

		if (SHOW_IMAGES) {
			cameraPane = new DisplayWindow("Camera output", width, height);
			opencvPane = new DisplayWindow("OpenCV output", width, height);
	
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
				cameraPane.updateWindow(rawImage);
				opencvPane.updateWindow(processedImage);
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
}


public class Main {
	public final static int PARTICLE_COUNT = 10; 	// How many samples of the world?
	public final static int PRUNED_COUNT = 5;		// How many samples do we keep at the end of each step?
	public final static double TRAVEL_STEP = 1;		// Inches / step
	public final static double TURN_STEP = 0.5;		// Radians / step
	
	public static void main(String[] args) {
		// Just a testing framework for the computer vision stuff.
		cvHandle handle = new cvHandle(); // Run the cv stuff.
		Thread cvThread = new Thread(handle);
		cvThread.start();
		
		cvData data = handle.data;
		BotClientMap map = BotClientMap.getDefaultMap();
		SparseGrid grid = new SparseGrid(data.gridSize, map, data.robotWidth);
		ArrayList<Pose> robotPositions = new ArrayList<Pose>();
		for (int i=0; i<PARTICLE_COUNT; i++) {
			robotPositions.add(map.startPose);
		}
		double normalization = 0;
		grid.writeMap();
		
		DisplayWindow cameraPane = new DisplayWindow("Derp", 600, 600);
		Random rng = new Random();

		while (true) {
			synchronized(handle.data) {
				data = handle.data;
				if (data.angles.size() == 0) {
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
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
							return -1;
						} else if (a.prob < b.prob) {
							return 1;
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
				System.out.println(normalization);
				grid.robotX = bestGuess.x;
				grid.robotY = bestGuess.y;
				grid.robotTheta = bestGuess.theta;
			}
			
			if (data.processedImage != null) {
				Mat finalMap = ImageProcessor.drawGrid(data.processedImage.size(), data, grid);
				cameraPane.updateWindow(finalMap);
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
