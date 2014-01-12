package edu.mit.felixsun.maslab;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class ImageProcessor {
	static int greenLowerH = 40;
	static int greenUpperH = 80;
	static int redLowerH = 170;
	static int redUpperH = 10;
	static int blueLowerH = 90;
	static int blueUpperH = 130;
	static int lowerS = 120;
	static int lowerV = 40;
	static double OK_RATIO = 2.0;
	static double MIN_FILL_PROPORTION = 0.2;
	static Scalar GREEN = new Scalar(0, 255, 0);
	static Scalar YELLOW = new Scalar(0,255,255);
	static Scalar RED = new Scalar(0,0,255);
	static Scalar BLUE = new Scalar(255,0,0);
	static double POLYAPPROXEPSILON = 30;
	static double MIN_GOOD_AREA = 200;
	//find this
	static double INCHESPERPIXEL = 1;
	
	static double VIEWANGLE = Math.PI/2;
	
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

	// Input: an image from the camera, an empty mat to store an output image (for
	// visual debugging only).
	// Output: A cvData structure, containing data that the main controller wants to know.
	public static void process(Mat rawImage, Mat processedImage, cvData data) {
		// Convert to HSV
		Mat hsvImage= new Mat();
		Imgproc.cvtColor(rawImage, hsvImage, Imgproc.COLOR_BGR2HSV);
		
		// Run each sub-processor in succession.
		// In the future, the robot controller may tell us to only do certain processes, to save
		// time.
		findBalls(hsvImage, new Mat(), data);
		findWalls(hsvImage, processedImage, blueLowerH, blueUpperH, data);
	}

	
	static void findBalls(Mat hsvImage, Mat processedImage, cvData data) {
		// Find balls - green for now.
		// Arbitrary colors coming soon.
		Mat colorMask = colorFilter(hsvImage, greenLowerH, greenUpperH);
		// Core.inRange(hsvImage, redFilterLower, redFilterUpper, colorMask);
		
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
		Imgproc.cvtColor(colorMask, processedImage, Imgproc.COLOR_GRAY2BGR);
		
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
		synchronized(data) {
			data.offset = offset;
		}
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

	static void findWalls(Mat hsvImage, Mat processedImage, int lowerHue, int upperHue, cvData data) {
		//Filter image by color
		Mat colorMask = new Mat();
		colorMask = colorFilter(hsvImage, lowerHue, upperHue);
		//Find the largest connected component
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Mat hierarchy = new Mat();
		Imgproc.findContours(colorMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
		double biggestArea = 0;
		int bestBlob = 0;
		double thisArea = 0;
		for(int i=0; i<contours.size(); i++){
			thisArea = Imgproc.boundingRect(contours.get(i)).area();
			if(thisArea >biggestArea){
				bestBlob = i;
				biggestArea = thisArea; 
			}
		}
		double bestArea = Imgproc.contourArea(contours.get(bestBlob));
		//Approximate the blob by a coarse polygon
		MatOfPoint2f polygon = new MatOfPoint2f();
		MatOfPoint2f strip = new MatOfPoint2f(contours.get(bestBlob).toArray());
		Imgproc.approxPolyDP(strip, polygon, POLYAPPROXEPSILON, true);
		
		MatOfPoint poly = new MatOfPoint(polygon.toArray());
		List<MatOfPoint> listy = new ArrayList<MatOfPoint>(); listy.add(poly);
		
		//Get the left and right heights of the polygon.
		//If the blob is well-behaved (which it should be during MASLAB),
		// these correspond to the leftmost and rightmost heights.
		//Left height := vertical distance between the two leftmost vertices.
		List<Point> polylist = poly.toList();
		int[] pts = leftMostTwo(polylist);
		Point pt1 = polylist.get(pts[0]);
		Point pt2 = polylist.get(pts[1]);
		double leftHeight = Math.abs(pt1.y-pt2.y);
		//Right height := vertical distance between the two rightmost vertices.
		int[] pts2 = rightMostTwo(polylist);
		Point pt3 = polylist.get(pts2[0]);
		Point pt4 = polylist.get(pts2[1]);
		double rightHeight = Math.abs(pt3.y-pt4.y);
		
		boolean onRight = false;
		double bigHeight = 0;
		if(rightHeight>leftHeight){
			onRight = true;
			bigHeight = rightHeight;
		}else{
			bigHeight = leftHeight;
		}
		//Get the other two points of the closest wall
		Point pt5 = pt1.clone();
		Point pt6 = pt1.clone();
		int[] closestWall = new int[2];
		if(onRight){
			closestWall = pts2;
			pt5 = polylist.get((closestWall[0]+1)%polylist.size());
			pt6 = polylist.get((closestWall[1]-1+polylist.size())%polylist.size());
		}
		else{
			closestWall = pts;
			pt5 = polylist.get((closestWall[0]-1+polylist.size())%polylist.size());
			pt6 = polylist.get((closestWall[1]+1)%polylist.size());
		}

		double smallHeight = pt5.y-pt6.y;
		
		//wallWidth is how wide the wall is on the image plane. 
		double wallWidth = Math.abs(polylist.get(closestWall[0]).x - pt5.x);
		double closeSideAngle = VIEWANGLE*(1 - polylist.get(closestWall[0]).x/hsvImage.width()) + (Math.PI - VIEWANGLE)/2;
		double farSideAngle = VIEWANGLE *(1- pt5.x/hsvImage.width()) + (Math.PI - VIEWANGLE)/2;
		//get angle between two sides of wall
		double wallAngle = VIEWANGLE * wallWidth/hsvImage.width();
		//distance to each side of wall in inches
		double closeDist = INCHESPERPIXEL * bigHeight;
		double farDist = INCHESPERPIXEL * smallHeight;
		//Length of wall: law of cosines
		double wallLength = Math.sqrt(Math.pow(closeDist,2)+ Math.pow(farDist,2) - 2*farDist*closeDist*Math.cos(wallAngle));
		//Direction of altitude to wall by law of sines
		double bearingAngle = Math.acos(farDist*Math.sin(wallAngle)/wallLength) + (Math.PI - VIEWANGLE)/2;
		
		/* data.wall is: 
		* (0,0,0) if no wall
		* (wallLength, bearing, x) if wall. (x = -1 if on left, 1 if wall on right) 
		*/
		
		double[] output = new double[] {0, 0, 0, 0, 0};
		if(Imgproc.contourArea(poly) > MIN_GOOD_AREA){
			output[0] = wallLength;
			output[1] = closeSideAngle;
			output[2] = farSideAngle;
			output[3] = bearingAngle;
			output[4] = onRight? 1:-1;
		}
		synchronized(data) {
			data.wall = output;
		}
		
	}

}
