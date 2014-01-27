package edu.mit.felixsun.maslab;

import devices.actuators.Cytron;
import devices.actuators.DigitalOutput;
import devices.actuators.Servo;
import devices.actuators.Servo1800A;
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
	public Servo sorter;
	public PWMOutput spiralPWM;
	public PWMOutput rollerPWM;
	public DigitalOutput spiralDirection;
	public DigitalOutput rollerDirection;
}