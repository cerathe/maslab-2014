package edu.mit.felixsun.maslab;

import java.awt.BorderLayout;
import java.awt.image.BufferedImage;
import java.util.Arrays;

import edu.mit.felixsun.maslab.ImageProcessor;
import edu.mit.felixsun.maslab.Mat2Image;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;

import jssc.SerialPort;
import jssc.SerialPortException;

class cvData {
	/*
	 * A bunch of data that gets passed between the computer vision system
	 * and the rest of the code.
	 */
	public double offset;
	public double[] wall;
	public cvData() {
		offset = -2;
		wall = new double[] {0, 0, 0, 0, 0};
	}
}

class cvHandle implements Runnable {
	/*
	 * Starts the cv scripts.  Runs in a separate thread.
	 */
	cvData data;
	Thread t;
	public cvHandle(cvData initData) {
		data = initData;
	}
	
	public void run(){
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		// Setup the camera
		VideoCapture camera = new VideoCapture();
		camera.open(1);
		
		// Create GUI windows to display camera output and OpenCV output
		int width = (int) (camera.get(Highgui.CV_CAP_PROP_FRAME_WIDTH));
		int height = (int) (camera.get(Highgui.CV_CAP_PROP_FRAME_HEIGHT));
		JLabel cameraPane = createWindow("Camera output", width, height);
		JLabel opencvPane = createWindow("OpenCV output", width, height);

		// Main loop
		Mat rawImage = new Mat();
		Mat processedImage = new Mat();
		while (true) {
			
			// Wait until the camera has a new frame
			camera.grab();
			camera.retrieve(rawImage);
			
			// Process the image however you like
			processedImage = Mat.zeros(rawImage.size(), rawImage.type());
			ImageProcessor.process(rawImage, processedImage, data);
			
			// Update the GUI windows
			updateWindow(cameraPane, rawImage);
			updateWindow(opencvPane, processedImage);
			// Manually garbage collect, because opencv has memory issues :(
			System.gc();
			
		}
	}
	
	private static JLabel createWindow(String name, int width, int height) {    
        JFrame imageFrame = new JFrame(name);
        imageFrame.setSize(width, height);
        imageFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        JLabel imagePane = new JLabel();
        imagePane.setLayout(new BorderLayout());
        imageFrame.setContentPane(imagePane);
        
        imageFrame.setVisible(true);
        return imagePane;
    }
    
    private static void updateWindow(JLabel imagePane, Mat mat) {
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
	//ball following
	public static int SPEED = 20;
	public static double I_GAIN = 0;
	
	//wall following
	public static double OPTIMALBEARING = 50;
	public static double WALLSPEED = 0.1;
	public static double I_GAIN_WALL = 0;

	public static void main(String[] args) {
		// Just a testing framework for the computer vision stuff.
		cvData data = new cvData();
		cvHandle handle = new cvHandle(data); // Run the cv stuff.
		Thread cvThread = new Thread(handle);
		cvThread.start();

		// Start serial communication.
		SerialPort serialPort;
		serialPort = new SerialPort("COM5");
		try {
            serialPort.openPort();
            serialPort.setParams(115200, 8, 1, 0);
		} catch (Exception e) {
            System.out.println(e);
            return;
		}

		while (true) {
			//for ball following
			double offset;
			double diff;
			double integral = 0;
			
			//for wall following
			double[] wall = new double[]{0, 0, 0, 0, 0}; 
			double walloffset = 0;
			double walldiff;
			double wallint = 0;
			
			int motorA = 0;
			int motorB = 0;
			synchronized(data){
				offset = data.offset;
				wall = data.wall;
			}
			if (offset < -1){
				// We don't see color.
				if(Arrays.equals(wall, new double[]{0, 0, 0, 0, 0})){
					//or a wall.
					motorA = 0;
					motorB = 0;
				}
				else{
					// we see a wall: steer proportional to your offset from the wall.
					double onRight = wall[4];
					double wallLength = wall[0];
					double bearing = wall[3];
					
					// if bearing > PI/2, then you're facing away from the wall.
					walloffset= onRight*(bearing-Math.PI/2);
					wallint += walloffset;
					
					walldiff = WALLSPEED*walloffset + I_GAIN_WALL*wallint; 
					motorA = (int) (SPEED + walldiff);
					motorB = (int) (SPEED - walldiff);
					
				}
				
			} else {
				// Steer proportional to where the color is.
				integral += offset;
				diff = SPEED*offset + I_GAIN*integral;
				motorA = (int) (SPEED - diff);
				motorB = (int) (SPEED + diff);
			}

			byte[] outData = new byte[4];
			outData[0] = 'S';				// Start signal "S"
			outData[1] = (byte) -motorA;	// Motor A data
			outData[2] = (byte) motorB;		// Motor B data
			outData[3] = 'E';				// End signal "E"
			try {
				serialPort.writeBytes(outData);
			} catch (SerialPortException e1) {
				e1.printStackTrace();
			}
			
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

	}

}
