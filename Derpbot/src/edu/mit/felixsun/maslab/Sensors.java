package edu.mit.felixsun.maslab;

import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.sensors.AnalogInput;
import devices.sensors.Encoder;

public class Sensors {
	public Encoder leftEncoder;
	public Encoder rightEncoder;
	public Cytron leftDriveMotor;
	public Cytron rightDriveMotor;
	public DigitalOutput led;
	public AnalogInput photoresistor;
}