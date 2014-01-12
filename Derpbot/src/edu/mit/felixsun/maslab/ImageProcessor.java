package edu.mit.felixsun.maslab;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class ImageProcessor {
	static int greenLowerH = 40;
	static int greenUpperH = 80;
	static int redLowerH = 170;
	static int redUpperH = 10;
	static int blueLowerH = 100;
	static int blueUpperH = 140;
	static int lowerS = 100;
	static int lowerV = 40;
	static double OK_RATIO = 2.0;
	static double MIN_FILL_PROPORTION = 0.2;
	static Scalar GREEN = new Scalar(0, 255, 0);
	
	// Camera and world parameters.  All length units in inches.
	static double wallStripeHeight = 11;
	
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
		return (282.3 / pixelHeight) * objectHeight;
	}

	// Input: an image from the camera, an empty mat to store an output image (for
	// visual debugging only).
	// Output: A cvData structure, containing data that the main controller wants to know.
	public static Mat process(Mat rawImage, cvData data) {
		Mat processedImage = new Mat();
		// Convert to HSV
		Mat hsvImage= new Mat();
		Imgproc.cvtColor(rawImage, hsvImage, Imgproc.COLOR_BGR2HSV);
		
		// Run each sub-processor in succession.
		// In the future, the robot controller may tell us to only do certain processes, to save
		// time.
		// findBalls(hsvImage, new Mat(), data);
		processedImage = findWalls(hsvImage, data);
		return processedImage;
	}

	
	static Mat findBalls(Mat hsvImage, cvData data) {
		// Find balls - green for now.
		// Arbitrary colors coming soon.
		Mat colorMask = colorFilter(hsvImage, greenLowerH, greenUpperH);
		
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
		return processedImage;
	}
	
	static Mat findWalls(Mat hsvImage, cvData data) {
		double scale = 10; // Pixels / inch
		Mat processedImage = Mat.zeros(hsvImage.size(), CvType.CV_8UC3);
		// Find all the wall stripes - TODO: make more general.
		Mat colorMask = colorFilter(hsvImage, blueLowerH, blueUpperH);
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
		synchronized(data) {
			data.grid = grid;
		}
		
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

}
