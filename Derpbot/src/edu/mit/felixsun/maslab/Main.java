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
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.Random;

import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.sensors.Encoder;
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
	HashMap<Entry<Double, Double>, Integer> landmarks;	// Holds (distance, angle) locations of balls, goals.
	/*
	 * Landmark key:
	 * 1 - ball
	 * 2 - opponent wall (center, yellow)
	 * More to come.
	 */
	public cvData() {
		offset = -2;
		angles = new HashMap<Double, Double>();
		landmarks = new HashMap<Entry<Double, Double>, Integer>();
	}
}

class Sensors {
	public Encoder leftEncoder;
	public Encoder rightEncoder;
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
	
	public static void main(String[] args) {
		// Just a testing framework for the computer vision stuff.
		cvHandle handle = new cvHandle(); // Run the cv stuff.
		Thread cvThread = new Thread(handle);
		cvThread.start();
		
		cvData data = handle.data;
		Localization localization = new Localization(data);
		Navigation navigation = new Navigation(localization);
		
		DisplayWindow cameraPane = new DisplayWindow("Derp", 600, 600);
		
		// Start serial communication.
		MapleComm comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		Sensors sensors = new Sensors();
		sensors.rightDriveMotor = new Cytron(2, 1);
		sensors.leftDriveMotor = new Cytron(7, 6);
		// Encoders: green - ground; blue - 5V; yellow - input A; white - input B.
		sensors.leftEncoder = new Encoder(29, 30);
		sensors.rightEncoder = new Encoder(31, 32);
		DigitalOutput ground1 = new DigitalOutput(0);
		DigitalOutput ground2 = new DigitalOutput(5);
		

		comm.registerDevice(sensors.leftDriveMotor);
		comm.registerDevice(sensors.rightDriveMotor);
		comm.registerDevice(sensors.leftEncoder);
		comm.registerDevice(sensors.rightEncoder);
		comm.registerDevice(ground1);
		comm.registerDevice(ground2);
		comm.initialize();
		
		ground1.setValue(false);
		ground2.setValue(false);

		comm.transmit();

		while (true) {
			comm.updateSensorData();
			synchronized(handle.data) {
				data = handle.data;
				Iterator<Entry<Double, Double>> iterBall = data.landmarks.keySet().iterator();
				while (iterBall.hasNext()) {
					Entry<Double, Double> thisBall = iterBall.next();
//					System.out.format("%f %f \n", thisBall.getKey(), thisBall.getValue());
				}
				localization.update(data, sensors);
//				navigation.drawPath(navigation.cleanUpNaive(navigation.naiveWallFollow(20,20, 100,75)));
//				navigation.drawPath((navigation.naiveWallFollow(70,40,100,75)));

//				navigation.loc.grid.drawList(navigation.straightLine(40,63,70,40));
//				System.out.println(navigation.loc.grid.getWallNeighbors(new SimpleEntry<Integer,Integer>(25,41)));
//				LinkedList<SimpleEntry<Integer,Integer>> blah = new LinkedList<SimpleEntry<Integer,Integer>>() ;
//				blah.add(new SimpleEntry<Integer,Integer>(25,128));
//				navigation.loc.grid.drawList(blah);
			}

			if (data.processedImage != null) {
				Mat finalMap = ImageProcessor.drawGrid(data.processedImage.size(), data, localization.grid);
				cameraPane.updateWindow(finalMap);
			}
			
			comm.transmit();
			
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
}
