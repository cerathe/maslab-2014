import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;


public class ImageProcessor {
	static Scalar greenFilterLower = new Scalar(40, 40, 40);
	static Scalar greenFilterUpper = new Scalar(80, 255, 255);
	static Scalar redFilterLower = new Scalar(0, 40, 40);
	static Scalar redFilterUpper = new Scalar(30, 255, 255);
	static Scalar colorFilterLower = redFilterLower;
	static Scalar colorFilterUpper = redFilterUpper;
	static double OK_RATIO = 2.0;
	static double MIN_FILL_PROPORTION = 0.2;
	static Scalar GREEN = new Scalar(0, 255, 0);
	
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	// Input: an image from the camera
	// Output: the OpenCV-processed image
	
	// (In practice it's a little different:
	//  the output image will be for your visual reference,
	//  but you will mainly want to output a list of the locations of detected objects.)
	public static Mat process(Mat rawImage) {
		// Convert to HSV
		Mat hsvImage= new Mat();
		Imgproc.cvtColor(rawImage, hsvImage, Imgproc.COLOR_BGR2HSV);
		// Find green stuff
		Mat colorMask = new Mat();
		Core.inRange(hsvImage, colorFilterLower, colorFilterUpper, colorMask);
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
		// Display the ball we found.
		Core.rectangle(rawImage, bestBoundingRect.tl(), bestBoundingRect.br(), GREEN);
		
		// Also show the masked input.
		Mat output = new Mat();
		Imgproc.cvtColor(colorMask, output, Imgproc.COLOR_GRAY2BGR);
//		hsvImage.release();
//		colorMask.release();
//		hierarchy.release();
//		contours.clear();
		return output;
	}
	
}
