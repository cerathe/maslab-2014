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
import devices.actuators.PWMOutput;
import devices.actuators.Servo1800A;
import devices.actuators.Servo3001HB;
import devices.sensors.AnalogInput;
import devices.sensors.Encoder;
import devices.sensors.Photoresistor;
import devices.sensors.Ultrasonic;
import edu.mit.felixsun.maslab.ImageProcessor;
import edu.mit.felixsun.maslab.Mat2Image;
import edu.mit.felixsun.maslab.SparseGrid.PointOfInterest;
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
import org.opencv.imgproc.Imgproc;

import BotClient.BotClient;
import comm.BotClientMap;
import comm.BotClientMap.Point;
import comm.BotClientMap.Pose;
import comm.BotClientMap.Wall;
import comm.CommInterface;
import comm.MapleComm;
import comm.SimMapleComm;
import comm.MapleIO;
import jssc.SerialPort;
import jssc.SerialPortException;

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
	final static boolean SIMULATE = false;
	final static boolean PRACTICE = true;
	public static void main(String[] args) {
		cvData data;
		// Just a testing framework for the computer vision stuff.
		cvHandle handle = new cvHandle(); // Run the cv stuff.
		Thread cvThread = new Thread(handle);
		if (!SIMULATE) {
			cvThread.start();
			data = handle.data;
		} else {
			data = new cvData();
		}

		DisplayWindow cameraPane = new DisplayWindow("Derp", 600, 600);
		
		Sensors sensors = new Sensors();
		sensors.rightDriveMotor = new Cytron(4, 5);
		sensors.leftDriveMotor = new Cytron(37, 0);
		// Encoders: green - ground; blue - 5V; yellow - input A; white - input B.
		sensors.leftEncoder = new Encoder(29, 30);
		sensors.rightEncoder = new Encoder(31, 32);
		DigitalOutput ground3 = new DigitalOutput(19);
		DigitalOutput ground4 = new DigitalOutput(20);
		
		sensors.photoresistor = new AnalogInput(17);
		sensors.redled = new DigitalOutput(18);
		sensors.greenled = new DigitalOutput(7);
		//TODO: Connect sorting servo actually and fix Servo type and pin.
		sensors.sorter = new Servo3001HB(12);
		sensors.rollerPWM = new PWMOutput(11);
		sensors.rollerDirection = new DigitalOutput(10);
		sensors.spiralPWM = new PWMOutput(14);
		sensors.spiralDirection = new DigitalOutput(13);
		sensors.rightDump = new Servo3001HB(2);
		sensors.leftDump = new Servo3001HB(9);
		// Start serial communication.
		CommInterface comm;
		if (!SIMULATE) {
			comm = new MapleComm(MapleIO.SerialPortType.WINDOWS);
		} else {
			comm = new SimMapleComm(null, sensors);
		}

		comm.registerDevice(sensors.leftDriveMotor);
		comm.registerDevice(sensors.rightDriveMotor);
		comm.registerDevice(sensors.leftEncoder);
		comm.registerDevice(sensors.rightEncoder);
		comm.registerDevice(ground3);
		comm.registerDevice(ground4);
		
		comm.registerDevice(sensors.photoresistor);
		comm.registerDevice(sensors.redled);
		comm.registerDevice(sensors.greenled);
		comm.registerDevice(sensors.sorter);
		comm.registerDevice(sensors.rollerDirection);
		comm.registerDevice(sensors.rollerPWM);
		comm.registerDevice(sensors.spiralDirection);
		comm.registerDevice(sensors.spiralPWM);
		comm.registerDevice(sensors.leftDump);
		comm.registerDevice(sensors.rightDump);
		comm.initialize();
		
		ground3.setValue(false);
		ground4.setValue(false);
		sensors.redled.setValue(true);
		sensors.greenled.setValue(true);
		sensors.rollerDirection.setValue(false);
		sensors.rollerPWM.setValue(1);
		sensors.spiralPWM.setValue(0.4);
		sensors.spiralDirection.setValue(true);
		sensors.sorter.setAngle((sensors.sorter.getMinAngle()+sensors.sorter.getMaxAngle())/2);
		sensors.leftDump.setAngle(sensors.leftDump.getMaxAngle());
		sensors.rightDump.setAngle(sensors.leftDump.getMinAngle());
		
		comm.transmit();
		
		BotClientMap map;
		BotClient botClient;
		if (SIMULATE || PRACTICE) {
			map = BotClientMap.getDefaultMap();
		} else {
			botClient = new BotClient("18.150.7.174:6667", "i5d76YlHmB", false);
			while (!botClient.gameStarted()) {
				System.out.println("Waiting...");
			}
			String mapString = botClient.getMap();
			map = new BotClientMap();
			map.load(mapString);
		}
		Localization localization = new Localization(data, map);
		Navigation navigation = new Navigation(localization);
		GamePlayState topState = new GamePlayState(navigation);
		BallSortState sort = new BallSortState(sensors);
		while (true) {
			comm.updateSensorData();
			synchronized(handle.data) {
				if (!SIMULATE) {
					data = handle.data;
				} else {
					data = comm.fakeImageProcessor();
				}
				localization.update(data, sensors);
			}
			topState.step(navigation, sensors);
			System.out.println(sensors.photoresistor.getValue());
			System.out.println(sort.step(1450, -105,860));
			
			Mat finalMap = ImageProcessor.drawGrid(new Size(600, 480), data, localization.grid);
			comm.transmit();
			cameraPane.updateWindow(finalMap);
			if (SIMULATE) {
			} else {
//				Mat smallMap = new Mat();
//				Imgproc.resize(finalMap, smallMap, new Size(320, 240));
//				BufferedImage out = Mat2Image.getImage(smallMap);
//				botClient.sendImage(out);
			}
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
}
