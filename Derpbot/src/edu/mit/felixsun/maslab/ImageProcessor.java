package edu.mit.felixsun.maslab;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;
import java.util.SortedMap;
import java.util.TreeMap;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;


public class ImageProcessor {
	static int greenLowerH = 40;
	static int greenUpperH = 80;
	static int tealLowerH = 80;
	static int tealUpperH = 95;
	static int redLowerH = 170;
	static int redUpperH = 10;
	static int blueLowerH = 100;
	static int blueUpperH = 120;
	static int yellowLowerH = 25;
	static int yellowUpperH = 35;
	static int lowerS = 60;
	static int lowerV = 40;
	static Scalar GREEN = new Scalar(0, 255, 0);
    static Scalar YELLOW = new Scalar(0,255,255);
    static Scalar RED = new Scalar(0,0,255);
    static Scalar BLUE = new Scalar(255,0,0);
    
    // Wall polygon approximation stuff.
    static double POLYAPPROXEPSILON = 3;
    static double MIN_GOOD_AREA = 200;
    	
	// Camera and world parameters.  All length units in inches.
	static double wallStripeHeight = 4;
	static double ballDiameter = 2;
    static double VIEWANGLE = 90 * Math.PI / 180;
	
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}
	
	static Mat colorFilter(Mat input, int lowerH, int upperH){
		/*
		 * Finds all pixels of input image (in HSV form) that have hue between 
		 * lowerH and upperH.  Is smart about H wrap-around: for example, if
		 * lowerH = 170 and upperH = 10, it will find lowerH between 170 and 180;
		 * as well as between 180 and 10.
		 */
		Mat output = new Mat();
		Scalar lowerHSV = new Scalar(lowerH, lowerS, lowerV);
		Scalar upperHSV = new Scalar(upperH, 255, 255);
		if (upperH > lowerH) {
			Core.inRange(input, lowerHSV, upperHSV, output);
		} else {
			Mat part1 = new Mat();
			Mat part2 = new Mat();
			Core.inRange(input, lowerHSV, new Scalar(180, 255, 255), part1);
			Core.inRange(input, new Scalar(0, lowerS, lowerV), upperHSV, part2);
			Core.bitwise_or(part1, part2, output);
		}
		return output;
	}
	
	static Mat findWhite(Mat input) {
		/*
		 * Experimental: Finds white blobs.
		 */
		Mat output = new Mat();
//		Core.inRange(input, new Scalar(0, 0, 140), new Scalar(180, 80, 255), output);
		Core.inRange(input, new Scalar(0, 0, 150), new Scalar(180, 80, 255), output);
		return output;
	}
	
	static double distanceConvert(double pixelHeight, double objectHeight, double angle) {
		/*
		 * Given an object that is objectHeight tall in the real world, and shows up as
		 * pixelHeight tall on camera, calculate how far away this object is from the camera.
		 */
		double FISHBOWL = 0.4;
		return (350.0 / pixelHeight) * objectHeight / (1 - Math.pow(Math.PI/2 - angle, 2) * FISHBOWL);
	}

	static double angularPosition(double xpos, double windowWidth){
		double ALPHA = 1;
		double ratio = (windowWidth - 2*xpos)/windowWidth;
		return Math.PI / 2 + (ALPHA * ratio - (1 - ALPHA) * Math.pow(ratio, 3)) * Math.PI / 4; 
		
	}
	// Input: an image from the camera, an empty mat to store an output image (for
	// visual debugging only).
	// Output: A cvData structure, containing data that the main controller wants to know.
	public static cvData process(Mat rawImage) {
		cvData data = new cvData();
		Mat processedImage = Mat.zeros(new Size(500,500), rawImage.type());
		// Convert to HSV
		Mat hsvImage= new Mat();
		Imgproc.cvtColor(rawImage, hsvImage, Imgproc.COLOR_BGR2HSV);		
		// Run each sub-processor in succession.
		// In the future, the robot controller may tell us to only do certain processes, to save
		// time.

		processedImage = findWallsPoly(hsvImage, data, 1);
		findBalls(hsvImage, data, greenLowerH, greenUpperH);
		findBalls(hsvImage, data, redLowerH, redUpperH);
		findStripe(hsvImage, data, tealLowerH, tealUpperH, 0);
		data.processedImage = processedImage;
		data.offset = 3;
		return data;
	}

	
	static Mat findBalls(Mat hsvImage, cvData data, int lowerH, int upperH) {
		int MIN_BALL_HEIGHT = 250;
		double BALL_RATIO = 2;
		double MIN_FILL_PROPORTION = 0.2;
		double MIN_BALL_AREA = 20;
	    // Find balls - both red and green in one shot.
	    Mat colorMask = colorFilter(hsvImage, lowerH, upperH);
		// Find blobs of color
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		// Also show the masked input.
		Mat processedImage = new Mat();
		Imgproc.cvtColor(colorMask, processedImage, Imgproc.COLOR_GRAY2BGR);
		Mat hierarchy = new Mat();
		Imgproc.findContours(colorMask.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
		// Look for the "best" blob
		// The best blob is lowest one that is
		// - Roughly square / circular
		// - Filled enough
		// - Large enough.
		int bestBlob = -1;
		Rect bestBoundingRect = new Rect();
		double bestHeight = 0;
		for (int i = 0; i < contours.size(); i++) {
			// Make a rectangle.
			Rect boundingRect = Imgproc.boundingRect(contours.get(i));
			// Is the rectangle roughly square?
			double height_ratio = 1.0 * boundingRect.width / boundingRect.height;
			if (! ((1.0 / BALL_RATIO < height_ratio) && (height_ratio < BALL_RATIO))){
				continue;
			}
			// Is the rectangle filled enough?
			double blobArea = Imgproc.contourArea(contours.get(i));
			if (1.0 * blobArea / boundingRect.area() < MIN_FILL_PROPORTION){
				continue;
			}
			// Is the rectangle large enough?
			if (blobArea < MIN_BALL_AREA || boundingRect.y < MIN_BALL_HEIGHT) {
				continue;
			}
			
			// If we get this far, we are good.  See if it is lower than the lowest so far.
			if (boundingRect.y > bestHeight){
				bestBlob = i;
				bestHeight = boundingRect.y;
				bestBoundingRect = boundingRect;
			}
		}
		// If we didn't find a ball, just stop now.
		if (bestBlob == -1) {
			return processedImage;
		}
		
		// Draw the ball on processedImage.
		Core.rectangle(processedImage, bestBoundingRect.tl(), bestBoundingRect.br(), GREEN);
		
		//Put the ball in the output data.
		double approxDiam = Math.sqrt(bestBoundingRect.area());
		double angularPos = angularPosition(bestBoundingRect.x+bestBoundingRect.width/2, hsvImage.width());
		double distance = distanceConvert(approxDiam, ballDiameter, angularPos);
		Entry<Double, Double> polarLoc = new SimpleEntry<Double, Double>(distance, angularPos);
		if (lowerH == greenLowerH) {
			data.ballGreenPolarLoc = polarLoc;
		} else {
			data.ballRedPolarLoc = polarLoc;
		}
		return processedImage;
	}
	
	static Mat findStripe(Mat hsvImage, cvData data, int lowerH, int upperH, int landmarkIndex) {
		// This is a potential wall stripe.
		// Is the rectangle filled enough?
		double MIN_STRIPE_AREA = 400;	// For red and green wall stripes.
		double COLORED_STRIPE_HEIGHT = 2;
		int MAX_STRIPE_HEIGHT = 250;
		double MIN_FILL_PROPORTION = 0.4;
		
		List<Entry<Double, Double>> stripes = new ArrayList<Entry<Double, Double>>();
		Mat colorMask = colorFilter(hsvImage, lowerH, upperH);
		// Find blobs of color
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		// Also show the masked input.
		Mat processedImage = new Mat();
		Imgproc.cvtColor(colorMask, processedImage, Imgproc.COLOR_GRAY2BGR);
		Mat hierarchy = new Mat();
		Imgproc.findContours(colorMask.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
		// Look for blobs that are
		// - Filled enough
		// - Large enough.
		for (int i = 0; i < contours.size(); i++) {
			Rect boundingRect = Imgproc.boundingRect(contours.get(i));
			double blobArea = Imgproc.contourArea(contours.get(i));
			if (1.0 * blobArea / boundingRect.area() < MIN_FILL_PROPORTION){
				continue;
			}
			// Is the rectangle large enough?
			if (blobArea < MIN_STRIPE_AREA) {
				continue;
			}
			// Calculate angle and distance.
			double angularPos = angularPosition(boundingRect.x+boundingRect.width/2, hsvImage.width());
			// OK, this is really hacky, but it should work.
			double shortHeight = boundingRect.height * (blobArea / boundingRect.area());
			double midHeight = (boundingRect.height + shortHeight) / 2;
			double distance = distanceConvert(midHeight, COLORED_STRIPE_HEIGHT, angularPos);
			Entry<Double, Double> polarLoc = new SimpleEntry<Double, Double>(distance, angularPos);
			stripes.add(polarLoc);
			Core.rectangle(processedImage, boundingRect.tl(), boundingRect.br(), GREEN);
		}

		data.landmarks.set(landmarkIndex, stripes);
		return processedImage;

	}
	
    //Linear regression
    private static double[] linReg(double[] x, double[] y){
    	double xybar = 0;
    	double xbar = 0;
    	double ybar = 0;
    	double x2bar = 0;
    	double n = x.length;
    	for(int i=0; i<x.length; i++){
    		xybar += x[i]*y[i]/n;
    		xbar += x[i]/n;
    		ybar += y[i]/n;
    		x2bar += x[i]*x[i]/n;
    	}
    	double b = (xybar - xbar*ybar)/(x2bar - xbar*xbar);
    	double a = ybar - b*xbar;
    	return new double[]{a,b};
    	
    }
    
	static Mat findWallsPoly(Mat hsvImage, cvData data, int value){
		/*
		 * Locates all the walls the camera can see.
		 * Dumps to data.angles a hashmap of angle -> distance to wall at that angle.
		 */
		final int COLUMN_MARGIN = 2;
		final int MIN_WALL_CENTER = hsvImage.height() / 2 - 50;
		Mat processedImage = Mat.zeros(hsvImage.size(), hsvImage.type());
		HashMap<Double, Double> angles = data.angles;
		
		//Filter image by color
        Mat colorMask = new Mat();
		colorMask = findWhite(hsvImage);
        //dilate and erode, to clean up lines a little.
		Imgproc.erode(colorMask, colorMask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3)),
				new Point(0, 0), 2);
		Imgproc.dilate(colorMask, colorMask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3)),
				new Point(0, 0), 2);
//		Imgproc.cvtColor(colorMask, processedImage, Imgproc.COLOR_GRAY2BGR);
//		if (5>4) {
//			return processedImage;
//		}
		//Separate into connected components
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(colorMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        
        if (contours.size() == 0) {
        	return processedImage;
        }
        
        // Sort the contours by vertical position.  Use only the lowest contour in each column of the picture.
        // This can be optimized by memoizing the bounding rectangles.  (Consider doing this if sorting becomes
        // a bottleneck.)
        Collections.sort(contours, new Comparator<MatOfPoint>(){
        	public int compare(MatOfPoint a, MatOfPoint b) {
        		Rect aRect = Imgproc.boundingRect(a);
        		Rect bRect = Imgproc.boundingRect(b);
        		return (bRect.y - bRect.height) - (aRect.y - aRect.height);
        	}
        });
        List<MatOfPoint> goodContours = new ArrayList<MatOfPoint>();
        boolean[] columnUsed = new boolean[hsvImage.width()];
        Arrays.fill(columnUsed, false);
        for (int i=0; i<contours.size(); i++) {
        	MatOfPoint thisContour = contours.get(i);
        	Rect contourRect = Imgproc.boundingRect(thisContour);
        	if (contourRect.y + contourRect.height / 2 < MIN_WALL_CENTER) {
        		continue;
        	}
        	boolean useMe = false;
        	int minColumn = contourRect.x - COLUMN_MARGIN;
        	if (minColumn < 0) {
        		minColumn = 0;
        	}
        	int maxColumn = contourRect.x + contourRect.width + COLUMN_MARGIN;
        	if (maxColumn > hsvImage.width()) {
        		maxColumn = hsvImage.width();
        	}
        	for (int column = minColumn; column < maxColumn; column++) {
        		if (!columnUsed[column]){
        			useMe = true;
        		}
    			columnUsed[column] = true;
        	}
        	if (useMe) {
        		goodContours.add(thisContour);
        	}
        }
        
        for (int i=0; i<goodContours.size(); i++) {
	        //Approximate the blob by a coarse polygon
	        MatOfPoint2f polygon = new MatOfPoint2f();
	        MatOfPoint2f strip = new MatOfPoint2f(goodContours.get(i).toArray());
	        Imgproc.approxPolyDP(strip, polygon, POLYAPPROXEPSILON, true);
	        MatOfPoint poly = new MatOfPoint(polygon.toArray());
	        MatOfPoint oldPoly = new MatOfPoint(polygon.toArray());
	        //Draw polygon to new image.
	        // To save space and time, make the polygon image smaller than the whole image.
	        Rect bounding = Imgproc.boundingRect(poly);
        	Mat polygonImage = Mat.zeros(new Size(bounding.width, bounding.height), processedImage.type());
        	for (int pointX = 0; pointX < poly.height(); pointX++) {
    			poly.put(pointX, 0, new double[] {poly.get(pointX, 0)[0] - bounding.x, poly.get(pointX, 0)[1] - bounding.y});
        	}
	        List<MatOfPoint> approxWall = new ArrayList<MatOfPoint>();
	        List<MatOfPoint> oldApproxWall = new ArrayList<MatOfPoint>();
	        approxWall.add(poly);
	        oldApproxWall.add(oldPoly);
	        Imgproc.drawContours(polygonImage, approxWall, 0, BLUE, -1);
	        Imgproc.drawContours(processedImage, oldApproxWall, 0, BLUE, -1);
	        
	        /* Figure out heights across the polygon. 
	        */
	        Mat heights = new Mat();
			Core.reduce(polygonImage, heights, 0, Core.REDUCE_SUM, CvType.CV_32S);
			for (int j = 0; j < heights.width(); j++) {
				
	            double thisHeight = heights.get(0, j)[0] / 255;
	            if (thisHeight < 10) {
	                    continue;
	            }
	            
	            double angle = angularPosition(bounding.x + j, processedImage.width());
	            double distance = distanceConvert(thisHeight, wallStripeHeight, angle);
	            if(angle<0.75*Math.PI && angle > 0.25*Math.PI){
//		            double wallX = Math.cos(angle)*distance;
//		            double wallY = Math.sin(angle)*distance + data.robotWidth/2;
	            	angles.put(angle, distance);
	            }
			}
        }
        
        return processedImage;
        
	}
	
	static public Mat drawGrid(Size size, cvData data, SparseGrid grid) {
		/*
		 * Draws a the grid found in data.grid.
		 * This will probably be moved to another class soon.
		 */
		double scale = 2;
		double offsetX = - grid.minX;
		double offsetY = - grid.minY;
		Point tl, br;
		class coordsToImgPoint {
			/*
			 * Takes in a Point in inches, returns a Point in pixels.
			 */
			double offsetX, offsetY, scale;
			public coordsToImgPoint(double x, double y, double s) {
				offsetX = x;
				offsetY = y;
				scale = s;
			}
			Point cvt(Point coords) {
				return new Point((offsetX + coords.x )* scale,
						(offsetY + coords.y) * scale);
			}
			
			Point cvt(double x, double y) {
				return new Point((offsetX + x) * scale,
						(offsetY + y) * scale);
			}
		}

        
		coordsToImgPoint converter = new coordsToImgPoint(offsetX, offsetY, scale);
		Mat processedImage = Mat.zeros(size, CvType.CV_8UC3);
        Iterator<Entry<Integer, Integer>> keys = grid.map.keySet().iterator();
        while (keys.hasNext()) {
            Entry<Integer, Integer> coords = keys.next();
            tl = converter.cvt(coords.getKey() * grid.gridSize, coords.getValue() * grid.gridSize);
            br = converter.cvt((coords.getKey() + 1) * grid.gridSize, (coords.getValue() + 1) * grid.gridSize);
            double value = grid.map.get(coords);

            switch((int)value){	
            	case 1: Core.rectangle(processedImage, tl, br, YELLOW); break;
            	case 2: Core.rectangle(processedImage, tl, br, RED); break;
            	case 3: Core.rectangle(processedImage, tl, br, GREEN); break;
            	default: Core.rectangle(processedImage, tl, br, BLUE); break;
            }
	    }
        
        double cameraX = grid.robotX + Constants.ROBOT_WIDTH/2 * Math.cos(grid.robotTheta);
        double cameraY = grid.robotY + Constants.ROBOT_WIDTH/2 * Math.sin(grid.robotTheta);
        //Draw ball.
        double angle = data.ballRedPolarLoc.getValue();
		double distance = data.ballRedPolarLoc.getKey();
		if (distance > 0) {
			double x = cameraX + distance * (Math.cos(grid.robotTheta + angle - Math.PI/2));
			double y = cameraY + distance * (Math.sin(grid.robotTheta + angle - Math.PI/2));
			tl = converter.cvt(x, y);
			br = converter.cvt(x + grid.gridSize, y + grid.gridSize);
			Core.rectangle(processedImage, tl, br, RED);
		}
		// Draw landmarks
		for (List<Entry<Double, Double>> landmarkType : data.landmarks) {
			if (landmarkType == null) {
				continue;
			}
			for (Entry<Double, Double> landmark : landmarkType) {
				angle = landmark.getValue();
				distance = landmark.getKey();
				double x = cameraX + distance * (Math.cos(grid.robotTheta + angle - Math.PI/2));
				double y = cameraY + distance * (Math.sin(grid.robotTheta + angle - Math.PI/2));
				tl = converter.cvt(x, y);
				br = converter.cvt(x + grid.gridSize, y + grid.gridSize);
				Core.rectangle(processedImage, tl, br, GREEN);
			}
		}
		// Draw the robot's camera stuff.
        HashMap<Double, Double> angles = data.angles;
        for (Entry<Double, Double> obs : angles.entrySet()) {
        	double wallX = cameraX + obs.getValue() * 
        			Math.cos(grid.robotTheta + obs.getKey() - Math.PI/2);
        	double wallY = cameraY + obs.getValue() * 
        			Math.sin(grid.robotTheta + obs.getKey() - Math.PI/2);
        	tl = converter.cvt(wallX, wallY);
        	br = converter.cvt(wallX + grid.gridSize, wallY + grid.gridSize);
        	Core.rectangle(processedImage, tl, br, YELLOW);
        }
        

        double gs = grid.gridSize;
        // And finally, draw the robot.
        Point robot = converter.cvt(grid.robotX * grid.gridSize, grid.robotY * grid.gridSize);
        Core.circle(processedImage, robot, (int) (Constants.ROBOT_WIDTH/2 * grid.gridSize * scale), YELLOW);
        Point robotVector = converter.cvt(grid.robotX + Constants.ROBOT_WIDTH * Math.cos(grid.robotTheta),
        		grid.robotY + Constants.ROBOT_WIDTH * Math.sin(grid.robotTheta));
        Core.line(processedImage, robot, robotVector, YELLOW);
        
        Mat finalImage = new Mat(new Size(0,processedImage.cols()), processedImage.type()); 
        for(int i =0; i<processedImage.rows(); i++){
        	finalImage.push_back(processedImage.row(processedImage.rows()-i-1));
        }
        
	    return finalImage;
	}

}
