package edu.mit.felixsun.maslab;

import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.actuators.Servo3001HB;
import devices.sensors.AnalogInput;
import devices.sensors.Encoder;
import devices.actuators.PWMOutput;

public class Sensors {
	public Encoder leftEncoder;
	public Encoder rightEncoder;
	public Cytron leftDriveMotor;
	public Cytron rightDriveMotor;
	public DigitalOutput led;
	public AnalogInput photoresistor;
	public Servo3001HB sorter;
	public Servo3001HB leftDump;
	public Servo3001HB rightDump;
	public PWMOutput spiralPWM;
	public PWMOutput rollerPWM;
	public DigitalOutput spiralDirection;
	public DigitalOutput rollerDirection;
}