package edu.mit.felixsun.maslab;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
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
	static int redLowerH = 170;
	static int redUpperH = 10;
	static int blueLowerH = 100;
	static int blueUpperH = 120;
	static int yellowLowerH = 25;
	static int yellowUpperH = 35;
	static int lowerS = 100;
	static int lowerV = 40;
	static double OK_RATIO = 2.0;
	static double MIN_FILL_PROPORTION = 0.2;
	static Scalar GREEN = new Scalar(0, 255, 0);
    static Scalar YELLOW = new Scalar(0,255,255);
    static Scalar RED = new Scalar(0,0,255);
    static Scalar BLUE = new Scalar(255,0,0);
    
    static double POLYAPPROXEPSILON = 3;
    static double MIN_GOOD_AREA = 200;
    	
	// Camera and world parameters.  All length units in inches.
	static double wallStripeHeight = 2;
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
	
	static double distanceConvert(double pixelHeight, double objectHeight) {
		/*
		 * Given an object that is objectHeight tall in the real world, and shows up as
		 * pixelHeight tall on camera, calculate how far away this object is from the camera.
		 */
		return (280.0 / pixelHeight) * objectHeight;
	}

	static double angularPosition(double xpos, double windowWidth){
		return (windowWidth - xpos)/windowWidth *VIEWANGLE + (Math.PI - VIEWANGLE)/2; 
		
	}
	// Input: an image from the camera, an empty mat to store an output image (for
	// visual debugging only).
	// Output: A cvData structure, containing data that the main controller wants to know.
	public static cvData process(Mat rawImage) {
		cvData data = new cvData();
		Mat processedImage = new Mat();
		// Convert to HSV
		Mat hsvImage= new Mat();
		Imgproc.cvtColor(rawImage, hsvImage, Imgproc.COLOR_BGR2HSV);
		// We will only see walls on the top half of our image.
		// Actually, our sample images don't reflect that right now.
		Rect roi = new Rect(0, 0, hsvImage.width(), hsvImage.height());	// Should be height/2
		Mat topHalf = new Mat(hsvImage, roi);
		
		// Run each sub-processor in succession.
		// In the future, the robot controller may tell us to only do certain processes, to save
		// time.

		processedImage = findWallsPoly(topHalf, blueLowerH, blueUpperH, data, 1);
//		 processedImage = findWallsPoly(topHalf, blueLowerH, blueUpperH, data, 1);
//		 findWallsPoly(topHalf, greenLowerH, greenUpperH, data, 3);
//		 findWallsPoly(topHalf, yellowLowerH, yellowUpperH, data, 4);
		findBalls(hsvImage, data);
		data.grid.removeIslands();
		// processedImage = drawGrid(hsvImage.size(), data);
		data.processedImage = processedImage;
		data.offset = 3;
		return data;
	}

	
	static Mat findBalls(Mat hsvImage, cvData data) {
		// Find balls - green for now.
		// Arbitrary colors coming soon.
		Mat colorMask = colorFilter(hsvImage, redLowerH, redUpperH);
		
		// Find blobs of color
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Mat hierarchy = new Mat();
		Imgproc.findContours(colorMask.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
		// Look for the "best" blob
		// The best blob is the largest one that is also
		// - Roughly square / circular
		// = Filled enough
		int bestBlob = -1;
		Rect bestBoundingRect = new Rect();
		double bestArea = 0;
		for (int i = 0; i < contours.size(); i++) {
			// Make a rectangle.
			Rect boundingRect = Imgproc.boundingRect(contours.get(i));
			// Is the rectangle roughly square?
			double height_ratio = 1.0 * boundingRect.width / boundingRect.height;
			if (! ((1.0 / OK_RATIO < height_ratio) && (height_ratio < OK_RATIO))){
				continue;
			}
			// Is the rectangle filled enough?
			double blobArea = Imgproc.contourArea(contours.get(i));
			if (1.0 * blobArea / boundingRect.area() < MIN_FILL_PROPORTION){
				continue;
			}
			// If we get this far, we are good.
			if (Imgproc.contourArea(contours.get(i)) > bestArea){
				bestBlob = i;
				bestArea = blobArea;
				bestBoundingRect = boundingRect;
			}
		}
		// Also show the masked input.
		Mat processedImage = new Mat();
		Imgproc.cvtColor(colorMask, processedImage, Imgproc.COLOR_GRAY2BGR);
		
		//Put the ball on the grid
		double approxDiam = Math.sqrt(bestArea);
		double angularPos = angularPosition(bestBoundingRect.x+bestBoundingRect.width/2, hsvImage.width());
		double distance = distanceConvert(approxDiam, ballDiameter);
		double wallX = Math.cos(angularPos)*distance;
		double wallY = Math.sin(angularPos)*distance;

		
		// How far off the center of the screen is the ball?
		double offset;
		if (bestBlob == -1) {
			// Actually, we couldn't even find the ball.  Report this.
			offset = -2;
		} else {
			// Offset:
			// -1 means all the way to the left
			// 1 means all the way to the right
			// 0 means centered
			offset = 1.0 * (bestBoundingRect.x + bestBoundingRect.width / 2 - hsvImage.width() / 2) / hsvImage.width();
		}
		
		data.offset = offset;
		data.grid.set(wallX, wallY, 2);
		double xx = bestBoundingRect.x;
		double yy = bestBoundingRect.y;
		double ww = bestBoundingRect.width;
		double hh = bestBoundingRect.height;
		Core.line(processedImage, new Point(xx,yy), new Point(xx + ww, yy+hh), RED);
		return processedImage;
	}
	
	static Mat findWalls(Mat hsvImage, cvData data) {
		/*
		 * NOT USED RIGHT NOW.
		 */
		double scale = 10; // Pixels / inch
		Mat processedImage = Mat.zeros(hsvImage.size(), CvType.CV_8UC3);
		// Find all the wall stripes - TODO: make more general.
		Mat colorMask = colorFilter(hsvImage, blueLowerH, blueUpperH);
		Mat kernel = Mat.ones(new Size(3, 3), CvType.CV_8U);
        Imgproc.erode(colorMask, colorMask, kernel, new Point(0, 0), 2);
        Imgproc.dilate(colorMask, colorMask, kernel, new Point(0, 0), 2);
		// Calculate heights.
		Mat heights = new Mat();
		Core.reduce(colorMask, heights, 0, Core.REDUCE_SUM, CvType.CV_32S);
		// Convert heights to distances, and plot walls on a map.
		SparseGrid grid = new SparseGrid(data.gridSize);
		for (int i = 0; i < heights.width(); i++) {
			double thisHeight = heights.get(0, i)[0] / 255;
			if (thisHeight < 2) {
				continue;
			}
			double distance = distanceConvert(thisHeight, wallStripeHeight);
			double angle = Math.PI/4*3 - Math.PI/2 * i / heights.width();
			double wallX = Math.cos(angle)*distance;
			double wallY = Math.sin(angle)*distance;
			grid.set(wallX, wallY, 1);	// 1 = generic blue wall.
		}
		
		// Shove new map into data
		data.grid = grid;
		
		// Draw the grid we just made.
		// Note: potential concurrency problems exist here, since the grid structure is
		// now shared between the vision and control threads.  However, this next section is
		// for visualization only, so it's OK if it messes up a little.
		Iterator<Entry<Integer, Integer>> keys = grid.map.keySet().iterator();
		// Oh God, that last line is so terrible.  I miss Python.
		while (keys.hasNext()) {
			Entry<Integer, Integer> coords = keys.next();
			Point tl = new Point(coords.getKey() * grid.gridSize * scale + processedImage.width() / 2, 
					coords.getValue() * grid.gridSize * scale);
			Point br = new Point((coords.getKey() + 1) * grid.gridSize * scale + processedImage.width() / 2, 
					(coords.getValue() + 1) * grid.gridSize * scale);
			Core.rectangle(processedImage, tl, br, GREEN);
		}
		Imgproc.cvtColor(colorMask, colorMask, Imgproc.COLOR_GRAY2BGR);
		return processedImage;
	}
	
    //Return the rightmost point in a List<Point>
    private static int[] rightMostTwo(List<Point> list){
            int best = 0;
            int secondBest = 0;
            for(int i=0; i<list.size(); i++){
                    if(list.get(i).x>list.get(best).x){
                            best = i;
                    }
                    else{
                            if(list.get(i).x>list.get(secondBest).x){
                                    secondBest=i;
                            }
                    }
            }
            if(list.get(best).y<list.get(secondBest).y){
                    return new int[]{best,secondBest};
            }
            else{
                    return new int[]{secondBest,best};
            }
    }
    
    //Return the leftmost point in a List<Point>
    private static int[] leftMostTwo(List<Point> list){
            int best = 0;
            int secondBest = 0;
            for(int i=0; i<list.size(); i++){
                    if(list.get(i).x<list.get(best).x){
                            best = i;
                    }
                    else{
                            if(list.get(i).x<list.get(secondBest).x){
                                    secondBest=i;
                            }
                    }
            }
            if(list.get(best).y<list.get(secondBest).y){
                    return new int[]{best,secondBest};
            }
            else{
                    return new int[]{secondBest,best};
            }
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
    
	static Mat findWallsPoly(Mat hsvImage, int lowerHue, int upperHue, cvData data, int value){
		int REGRESSION_SIZE = 10;
		Mat processedImage = Mat.zeros(hsvImage.size(), hsvImage.type());
		
		//Filter image by color
        Mat colorMask = new Mat();
        colorMask = colorFilter(hsvImage, lowerHue, upperHue);
        //dilate and erode - These don't appear to help right now, so let's not do them.
         Imgproc.dilate(colorMask, colorMask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3)));
         Imgproc.erode(colorMask, colorMask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3)));

        
        //Find the largest connected component
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        List<MatOfPoint> goodContours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(colorMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double biggestArea = 0;
        int bestBlob = -1;
        double thisArea = 0;
        // TODO: Figure out which blobs are walls.
//        for(int i=0; i<contours.size(); i++){
//        	if (looksLikeWall(contours.get(i))) {
//        		
//        	}
//                thisArea = Imgproc.boundingRect(contours.get(i)).area();
//                Rect bounding = Imgproc.boundingRect(contours.get(i));
//                
//        }
        goodContours = contours;
        
        
        if (goodContours.size() == 0) {
        	return processedImage;
        }
        SparseGrid grid = data.grid;
        
        SortedMap<Double, Point> distToPoints = new TreeMap<Double, Point>();
        for (int i=0; i<goodContours.size(); i++) {
	        //Approximate the blob by a coarse polygon
	        MatOfPoint2f polygon = new MatOfPoint2f();
	        MatOfPoint2f strip = new MatOfPoint2f(goodContours.get(i).toArray());
	        Imgproc.approxPolyDP(strip, polygon, POLYAPPROXEPSILON, true);
	        MatOfPoint poly = new MatOfPoint(polygon.toArray());
	        //Draw polygon to new image.
	        // To save space and time, make the polygon image smaller than the whole image.
	        Rect bounding = Imgproc.boundingRect(poly);
        	Mat polygonImage = Mat.zeros(new Size(bounding.width, bounding.height), processedImage.type());
        	for (int pointX = 0; pointX < poly.height(); pointX++) {
    			poly.put(pointX, 0, new double[] {poly.get(pointX, 0)[0] - bounding.x, poly.get(pointX, 0)[1] - bounding.y});
        	}
	        List<MatOfPoint> approxWall = new ArrayList<MatOfPoint>();
	        approxWall.add(poly);
	        Imgproc.drawContours(polygonImage, approxWall, 0, BLUE, -1);
	        
	        /* Figure out heights across the polygon. 
	        * Also keep the leftmost and rightmost 3 points.
	        * These points are used to extrapolate 
	        * via linear fitting the walls on the sides.
	        */
	        Mat heights = new Mat();
			Core.reduce(polygonImage, heights, 0, Core.REDUCE_SUM, CvType.CV_32S);
			for (int j = 0; j < heights.width(); j++) {
				
	            double thisHeight = heights.get(0, j)[0] / 255;
	            if (thisHeight < 2) {
	                    continue;
	            }
	            
	            double distance = distanceConvert(thisHeight, wallStripeHeight);
	            double angle = angularPosition(bounding.x + j, processedImage.width());
	            if(angle<0.75*Math.PI && angle > 0.25*Math.PI){
		            double wallX = Math.cos(angle)*distance;
		            double wallY = Math.sin(angle)*distance;
	            	grid.set(wallX, wallY, value);
	            }
			}
        }
        grid.removeIslands();
        // Make distToPoints:
      	Enumeration<Entry<Integer, Integer>> keys = grid.map.keys();
        while(keys.hasMoreElements()){
        	Entry<Integer, Integer> coords = keys.nextElement();
        	double wallX = grid.gridSize * coords.getKey();
        	double wallY = grid.gridSize * coords.getValue();
        	
	        Point thisPt = new Point(wallX, wallY);
	        double theta = Math.atan(wallY/wallX)>0? Math.atan(wallY/wallX): Math.PI - Math.abs(Math.atan(wallY/wallX));
	    	distToPoints.put(theta, thisPt);
        }
        // Get the leftmost n points and the rightmost n points.
        double[] leftWallX = new double[REGRESSION_SIZE];
        double[] rightWallX = new double[REGRESSION_SIZE];
        double[] leftWallY = new double[REGRESSION_SIZE];
        double[] rightWallY = new double[REGRESSION_SIZE];
        
        // Not enough points to do regression.  Stop.
        if (distToPoints.size() < 2*REGRESSION_SIZE) {
        	return processedImage;
        }
        	
        
        for (int i = 0; i < REGRESSION_SIZE; i++) {
        	double leftX = distToPoints.firstKey();
        	double rightX = distToPoints.lastKey();
        	Point leftPt = distToPoints.get(leftX);
        	Point rightPt = distToPoints.get(rightX);
        	distToPoints.remove(leftX);
        	distToPoints.remove(rightX);
        	leftWallX[i] = leftPt.x;
        	leftWallY[i] = leftPt.y;
        	rightWallX[i] = rightPt.x;
        	rightWallY[i] = rightPt.y;
        }
        
        //linear regression
        double[] leftCoeffs = linReg(leftWallX, leftWallY);
        double[] rightCoeffs = linReg(rightWallX, rightWallY);
        double ldist = Math.abs( leftCoeffs[0]/ (Math.sqrt(1 + (leftCoeffs[1] * leftCoeffs[1]))));
        double rdist = Math.abs( rightCoeffs[0]/ (Math.sqrt(1 + (rightCoeffs[1] * rightCoeffs[1]))));
        
        for (double yExtrapolate = leftWallY[0]; yExtrapolate > -10; yExtrapolate -= grid.gridSize) {
        	double xExtrapolate = (yExtrapolate - leftCoeffs[0]) / leftCoeffs[1];
        	if (yExtrapolate > leftWallY[0] && grid.filled(xExtrapolate, yExtrapolate)) {
        		// This means we are extrapolating in the wrong direction, and should stop!
        		break;
        	}
        	//grid.set(xExtrapolate, yExtrapolate, 2);
        }
        
        for (double yExtrapolate = rightWallY[0]; yExtrapolate > -10; yExtrapolate -= grid.gridSize) {
        	double xExtrapolate = (yExtrapolate - rightCoeffs[0]) / rightCoeffs[1];
        	if (yExtrapolate > rightWallY[0] && grid.filled(xExtrapolate, yExtrapolate)) {
        		break;
        	}
        	//grid.set(xExtrapolate, yExtrapolate, 2);
        }
        
        double[] output = new double[] {0, 0, 0, 0, 0};
        double bearingAngle = Math.atan(1/Math.abs(rightCoeffs[1]));
        output[0] = rdist;
        output[1] = rightCoeffs[0];//closeSideAngle;
        output[2] = rightCoeffs[1];//farSideAngle;
        output[3] = bearingAngle;//>Math.PI? bearingAngle-Math.PI: bearingAngle;
        output[4] = 1;//onRight? 1:-1;
        
        data.wall = output;
        data.grid = grid;
        return processedImage;
        
	}
	
	static Mat drawGrid(Size size, cvData data){
		double scale = 10;
		SparseGrid grid = data.grid;
		double wallDist, bearingAngle, onRight;
		double[] wall = data.wall;
		wallDist = wall[0]; bearingAngle = wall[3]; onRight = wall[4];
		double a = wall[1]; double b = wall[2];
		Mat processedImage = Mat.zeros(size, CvType.CV_8UC3);
        Iterator<Entry<Integer, Integer>> keys = grid.map.keySet().iterator();
        
        while (keys.hasNext()) {
            Entry<Integer, Integer> coords = keys.next();
            Point tl = new Point(coords.getKey() * grid.gridSize * scale + processedImage.width() / 2, 
                            coords.getValue() * grid.gridSize * scale);
            Point br = new Point((coords.getKey() + 1) * grid.gridSize * scale + processedImage.width() / 2, 
                            (coords.getValue() + 1) * grid.gridSize * scale);
            double value = grid.map.get(coords);
            switch((int)value){	
            	case 1: Core.rectangle(processedImage, tl, br, BLUE); break;
            	case 2: Core.rectangle(processedImage, tl, br, RED); break;
            	case 3: Core.rectangle(processedImage, tl, br, GREEN); break;
            	case 4: Core.rectangle(processedImage, tl, br, YELLOW); break;
            }
	    }
        double theta = onRight==1? bearingAngle: Math.PI - bearingAngle;
        theta = bearingAngle;
        //theta = closeSideAngle;
        double d = wallDist;
        double gs = grid.gridSize;
        Point p1 = new Point(processedImage.width()/2, 1);
        Point p2 = new Point(processedImage.width()/2 + scale*d*Math.cos(theta)/gs, scale*d*Math.sin(theta)/gs);
        Point p3 = new Point(processedImage.width()/2,  scale*a/gs);
        Point p4 = new Point(processedImage.width()/2 + scale*d*Math.cos(theta)/gs, scale*a/gs - b*(scale*d*Math.cos(theta))/gs);
        Core.line(processedImage,p1,p2, RED);
        Core.line(processedImage, p3, p4, GREEN);
        
        
        Mat finalImage = new Mat(new Size(0,processedImage.cols()), processedImage.type()); 
        for(int i =0; i<processedImage.rows(); i++){
        	finalImage.push_back(processedImage.row(processedImage.rows()-i-1));
        }
        
	    return finalImage;
	}

}
