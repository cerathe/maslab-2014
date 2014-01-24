package comm;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.opencv.core.Mat;
import org.opencv.core.Size;

import comm.MapleIO.SerialPortType;
import devices.MapleDevice;
import edu.mit.felixsun.maslab.DisplayWindow;
import edu.mit.felixsun.maslab.ImageProcessor;
import edu.mit.felixsun.maslab.SparseGrid;
import edu.mit.felixsun.maslab.Sensors;
import edu.mit.felixsun.maslab.Constants;
import edu.mit.felixsun.maslab.cvData;

public class SimMapleComm implements CommInterface {
	private List<MapleDevice> deviceList = new ArrayList<MapleDevice>();
	final double LEFT_MOTOR_BIAS = 12;	// Radians/s / motor unit
	final double RIGHT_MOTOR_BIAS = 10;	// Radians/s / motor unit
	final double FRACTION_WHEEL_VARIATION = 0.2;	// Average error of wheel motion.
	final double TIMESTEP = 0.1;		// seconds
	SparseGrid sim;
	Random rng = new Random();
	double leftEncoder;
	double rightEncoder;
	DisplayWindow cameraPane;
	Sensors sensors;

	public SimMapleComm(SerialPortType portType, Sensors sensors) {
		// Make the simulated world.
		sim = new SparseGrid(1, BotClientMap.getDefaultMap());
		sim.writeMap();
		// Fiddle with the robot starting position a little.
		sim.robotX += rng.nextGaussian() * 2;
		sim.robotY += rng.nextGaussian() * 2;
		sim.robotTheta += rng.nextGaussian() * 0.1;
		this.sensors = sensors;
		cameraPane = new DisplayWindow("Simulator Output", 600, 600);
		
	}
	
	public void initialize() {
		// Do nothing :)
	}
	
	public void transmit() {
		/*
		 * We cheat a little by taking in a Sensor object.
		 */
		leftEncoder = -sensors.leftDriveMotor.lastSet * LEFT_MOTOR_BIAS;
		rightEncoder = -sensors.rightDriveMotor.lastSet * RIGHT_MOTOR_BIAS;
		double leftD = leftEncoder * Constants.WHEEL_RADIUS * (1 + FRACTION_WHEEL_VARIATION * rng.nextGaussian());
		double rightD = rightEncoder * Constants.WHEEL_RADIUS * (1 + FRACTION_WHEEL_VARIATION * rng.nextGaussian());
		double deltaForward = (leftD + rightD) / 2 * TIMESTEP;
		double deltaTurn = (rightD - leftD) / Constants.WHEELBASE_WIDTH;
		double deltaX = deltaForward * Math.cos(sim.robotTheta);
		double deltaY = deltaForward * Math.sin(sim.robotTheta);
		sim.robotX += deltaX;
		sim.robotY += deltaY;
		sim.robotTheta += deltaTurn;
		
		// Now, draw a map
		Mat image = ImageProcessor.drawGrid(new Size(600, 600), new cvData(), sim);
		cameraPane.updateWindow(image);
	}
	
	public void updateSensorData() {
		sensors.leftEncoder.fakeUpdate(leftEncoder);
		sensors.rightEncoder.fakeUpdate(rightEncoder);
		
		// Add more sensors (gyro, ultrasound, etc.) here.
	}
	
	public cvData fakeImageProcessor() {
		cvData data = new cvData();
		for (double angle = Math.PI/4; angle < 3*Math.PI/4; angle += 0.01) {
			double dist = sim.trueMeas(angle, sim.robotX, sim.robotY, sim.robotTheta, 84);
			if (dist < 84) {
				data.angles.put(angle, dist);
			}
		}
		return data;
	}

	public void registerDevice(MapleDevice device) {
		// TODO Auto-generated method stub
		
	}
	
	
	

}
