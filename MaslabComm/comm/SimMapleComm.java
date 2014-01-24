package comm;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;
import java.util.Map.Entry;
import java.util.AbstractMap.SimpleEntry;

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
	private List<Entry<Double, Double>> balls = new ArrayList<Entry<Double, Double>>();
	final double LEFT_MOTOR_BIAS = 5;	// Radians/s / motor unit
	final double RIGHT_MOTOR_BIAS = 6;	// Radians/s / motor unit
	final double FRACTION_WHEEL_VARIATION = 0.2;	// Average error of wheel motion.
	final double TIMESTEP = 0.1;		// seconds
	final double TURN_DISCOUNT = 0.8;
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
		balls.add(new SimpleEntry<Double, Double>(30.0, 40.0));
		balls.add(new SimpleEntry<Double, Double>(55.0, 40.0));
		cameraPane = new DisplayWindow("Simulator Output", 600, 600);
		
	}
	
	public void initialize() {
		// Do nothing :)
	}
	
	public void transmit() {
		/*
		 * We cheat a little by taking in a Sensor object.
		 */
		leftEncoder = sensors.leftDriveMotor.lastSet * LEFT_MOTOR_BIAS * TIMESTEP;
		rightEncoder = sensors.rightDriveMotor.lastSet * RIGHT_MOTOR_BIAS * TIMESTEP;
		double leftD = -leftEncoder * Constants.WHEEL_RADIUS * (1 + FRACTION_WHEEL_VARIATION * rng.nextGaussian());
		double rightD = rightEncoder * Constants.WHEEL_RADIUS * (1 + FRACTION_WHEEL_VARIATION * rng.nextGaussian());
		double deltaForward = (leftD + rightD) / 2;
		double deltaTurn = (rightD - leftD) / Constants.WHEELBASE_WIDTH * TURN_DISCOUNT;
		double deltaX = deltaForward * Math.cos(sim.robotTheta);
		double deltaY = deltaForward * Math.sin(sim.robotTheta);
		// Check for wall collision
		if (! (sim.closestOccupied(sim.robotX + deltaX, sim.robotY + deltaY) < Constants.ROBOT_WIDTH/2)) {
			sim.robotX += deltaX;
			sim.robotY += deltaY;
			sim.robotTheta += deltaTurn;
		} else {
			leftEncoder = 0;
			rightEncoder = 0;
		}
		
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
		double cameraX = sim.robotX + Constants.ROBOT_WIDTH / 2 * Math.cos(sim.robotTheta);
		double cameraY = sim.robotY + Constants.ROBOT_WIDTH / 2 * Math.sin(sim.robotTheta);
		for (double angle = Math.PI/4; angle < 3*Math.PI/4; angle += 0.01) {
			double dist = sim.trueMeas(angle, cameraX, cameraY, sim.robotTheta, 84);
			if (dist < 84) {
				// "Fishbowl" effect.
				double distortedDist = dist * (1 - Math.pow(Math.PI/2 - angle, 2) / 2) + rng.nextGaussian()*2;
				data.angles.put(angle, distortedDist);
			}
		}
		
		// Add a ball.
		Entry<Double, Double> bestBallPolar = new SimpleEntry<Double, Double>(1000.0, -1.0);
		int removeIndex = -1;
		for (Entry<Double, Double> ball : balls) {
			double ballX = ball.getKey() - cameraX;
			double ballY = ball.getValue() - cameraY;
			double dist = sim.dist(0, 0, ballX, ballY);
			if (dist < 2) {
				System.out.println("You got the ball!");
				removeIndex = balls.indexOf(ball);
			}
			double ballAngle = Math.atan2(ballY, ballX) - sim.robotTheta;
			ballAngle = Math.atan2(Math.sin(ballAngle), Math.cos(ballAngle)) + Math.PI/2;
			// Only consider a ball if it is within the camera field-of-view and isn't obstructed by a wall.
			if ((ballAngle > Math.PI/4) && (ballAngle < 3*Math.PI/4) &&
					sim.trueMeas(ballAngle, cameraX, cameraY, -Math.PI/2, dist+1) > dist) {
				if (dist < bestBallPolar.getKey()) {
					bestBallPolar = new SimpleEntry<Double, Double>(dist, ballAngle);					
				}
			}	
		}
		if (bestBallPolar.getKey() < 1000) {
			data.ballPolarLoc = bestBallPolar;
		}
		if (removeIndex != -1) {
			balls.remove(removeIndex);
		}
		return data;
	}

	public void registerDevice(MapleDevice device) {
		// TODO Auto-generated method stub
		
	}
	
	
	

}
