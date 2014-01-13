package edu.mit.felixsun.maslab;

import java.awt.BorderLayout;
import java.awt.image.BufferedImage;
import java.util.AbstractMap.SimpleEntry;
import java.util.Arrays;
import java.util.HashMap;

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
	public SparseGrid grid;
	public double gridSize = 0.5;
	public double[] wall;
	public cvData() {
		offset = -2;
		grid = new SparseGrid(gridSize);
		wall = new double[] {0,0,0,0,0};
	}
}

class SparseGrid {
	/*
	 * A sparse-matrix style 2D grid, that stores an integer at each point.
	 */
	public double gridSize; // Inches per grid square
	public HashMap<java.util.Map.Entry<Integer, Integer>, Integer> map;

	public SparseGrid(double scale) {
		gridSize = scale;
		map = new HashMap();
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
		return map.get(coords);
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
//		VideoCapture camera = new VideoCapture();
//		camera.open(1);
		
		// Create GUI windows to display camera output and OpenCV output
//		int width = (int) (camera.get(Highgui.CV_CAP_PROP_FRAME_WIDTH));
//		int height = (int) (camera.get(Highgui.CV_CAP_PROP_FRAME_HEIGHT));
		Mat img = Highgui.imread("/Users/vipul/git/maslab-2014/Derpbot/src/edu/mit/felixsun/maslab/walls.png", Highgui.CV_LOAD_IMAGE_COLOR); 
		int width = img.width();
		int height = img.height();
		JLabel cameraPane = createWindow("Camera output", width, height);
		JLabel opencvPane = createWindow("OpenCV output", width, height);

		// Main loop
		Mat rawImage = Highgui.imread("/Users/vipul/git/maslab-2014/Derpbot/src/edu/mit/felixsun/maslab/wallsandballswithscribbles.png", Highgui.CV_LOAD_IMAGE_COLOR); 
;
		Mat processedImage = new Mat();
		while (true) {
			
			// Wait until the camera has a new frame
//			camera.grab();
//			camera.retrieve(rawImage);
			
			// Process the image however you like
			processedImage = ImageProcessor.process(rawImage, data);
			
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
	public static double I_GAIN = 0;
    public static double WALLSPEED = 0.1;
    public static double I_GAIN_WALL = 0;
    public static double TOOCLOSE = 10;
    
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
                            double wallDist = wall[0];
                            double bearing = wall[3];
                            if(wallDist>TOOCLOSE){
                            	motorA = (int) SPEED;
                            	motorB = (int) SPEED;
                            	System.out.println(wallDist);
                            }
                            else{
                            	System.out.println("TOOCLOSE");
                            	motorA = (int) -onRight* SPEED;
                            	motorB = (int) onRight * SPEED;
                            }
                            
//                            
//                            // if bearing > PI/2, then you're facing away from the wall.
//                            walloffset= onRight*(bearing-Math.PI/2);
//                            wallint += walloffset;
//                            
//                            walldiff = WALLSPEED*walloffset + I_GAIN_WALL*wallint; 
//                            motorA = (int) (SPEED + walldiff);
//                            motorB = (int) (SPEED - walldiff);
//                            
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
