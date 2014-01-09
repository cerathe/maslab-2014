package edu.mit.felixsun.maslab;

import java.awt.BorderLayout;
import java.awt.image.BufferedImage;

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
	public cvData() {
		offset = -1;
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
		camera.open(2);
		
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
	public static int SPEED = 20;

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
			double offset;
			int motorA = 0;
			int motorB = 0;
			synchronized(data){
				offset = data.offset;
			}
			System.out.println(offset);
			if (offset < 0){
				// We don't see color.  Just keep spinning.
				motorA = 0;
				motorB = 0;
			} else {
				// Steer proportional to where the color is.
				motorA = (int) (SPEED + offset*SPEED*2);
				motorB = (int) (20 - offset*SPEED*2);
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
