package edu.mit.felixsun.maslab;

import jssc.SerialPort;
import jssc.SerialPortException;

public class Main {

	public static void main(String[] args) {
		// Just a testing framework for the machine vision stuff.
		SerialPort serialPort;
		serialPort = new SerialPort("COM4");
		try {
            serialPort.openPort();
            serialPort.setParams(115200, 8, 1, 0);
		} catch (Exception e) {
            System.out.println(e);
            return;
		}

		while (true) {
			int motorA = 20;
			int motorB = 20;
			byte[] outData = new byte[4];
			outData[0] = 'S';				// Start signal "S"
			outData[1] = (byte) -motorA;	// Motor A data
			outData[2] = (byte) motorB;		// Motor B data
			outData[3] = 'E';				// End signal "E"
			try {
				serialPort.writeBytes(outData);
			} catch (SerialPortException e1) {
				e1.printStackTrace();
			}
			
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

	}

}
