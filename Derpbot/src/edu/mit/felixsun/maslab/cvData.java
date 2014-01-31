package edu.mit.felixsun.maslab;

import java.util.HashMap;
import java.util.Map.Entry;
import java.util.List;
import java.util.ArrayList;
import java.util.AbstractMap.SimpleEntry;

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
	// public HashMap<Entry<Double, Double>, Integer> landmarks = new HashMap<Entry<Double, Double>, Integer>();	// Holds (distance, angle) locations of goals.  Unused.
	public Entry<Double, Double> ballRedPolarLoc;
	public Entry<Double, Double> ballGreenPolarLoc;
	public List<List<Entry<Double, Double>>> landmarks;
	/*
	 * Landmark key:
	 * 1 - ball
	 * 2 - opponent wall (center, yellow)
	 * More to come.
	 */
	public cvData() {
		offset = -2;
		angles = new HashMap<Double, Double>();
		ballRedPolarLoc = new SimpleEntry<Double, Double>(-1.0, -1.0);
		ballGreenPolarLoc = new SimpleEntry<Double, Double>(-1.0, -1.0);
		landmarks = new ArrayList<List<Entry<Double, Double>>>();
		for (int i=0; i<3; i++) {
			// 0 - reactor (green)
			// 1 - opponent wall (yellow) - NOT USED?
			landmarks.add(null);
		}
	}
}