import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;


public class ImageProcessor {
	static Scalar greenFilterLower = new Scalar(40, 60, 60);
	static Scalar greenFilterUpper = new Scalar(80, 255, 255);
	
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	// Input: an image from the camera
	// Output: the OpenCV-processed image
	
	// (In practice it's a little different:
	//  the output image will be for your visual reference,
	//  but you will mainly want to output a list of the locations of detected objects.)
	public static Mat process(Mat rawImage) {
		Mat hsvImage= new Mat();
		Imgproc.cvtColor(rawImage, hsvImage, Imgproc.COLOR_BGR2HSV);
		Mat greenMask = new Mat();
		Core.inRange(hsvImage, greenFilterLower, greenFilterUpper, greenMask);
		Mat output = new Mat();
		Imgproc.cvtColor(greenMask, output, Imgproc.COLOR_GRAY2BGR);
		return output;
	}
	
}
