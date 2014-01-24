package edu.mit.felixsun.maslab;

import java.util.HashMap;
import java.util.Map.Entry;

import org.opencv.core.Mat;


public class cvData {
	/*
	 * A bunch of data that gets passed between the computer vision system
	 * and the rest of the code.
	 */
	public double offset;
	public double gridSize = 1;		// Inches / square
	public Mat processedImage;
	public HashMap<Double, Double> angles;
	public HashMap<Entry<Double, Double>, Integer> landmarks;	// Holds (distance, angle) locations of balls, goals.
	/*
	 * Landmark key:
	 * 1 - ball
	 * 2 - opponent wall (center, yellow)
	 * More to come.
	 */
	public cvData() {
		offset = -2;
		angles = new HashMap<Double, Double>();
		landmarks = new HashMap<Entry<Double, Double>, Integer>();
	}
}