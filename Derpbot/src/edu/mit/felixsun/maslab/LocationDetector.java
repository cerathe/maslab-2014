package edu.mit.felixsun.maslab;

import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.TreeSet;

import org.opencv.core.Mat;

import comm.BotClientMap;
import comm.BotClientMap.Pose;

public class LocationDetector {
	public TreeSet<Pose> poses;
	public cvData data;

	//precision = size of searched grid
	public static double xPrecision = 1;
	public static double thetaPrecision = 0.04;
	public static double xRange = 5;
	public static double thetaRange = 0.1;
	
	//How likely must a point be to be counted
	public static double pThresh = 0.001;
	
	public LocationDetector(cvData dat){
		poses.add(dat.map.startPose);
		data = dat;
	}
	
	/* Poses maintains a linked list of potential positions.
	 * 
	 * Every Iteration:
	 * push onto the heap some set of shifted positions with suitable probability.
	 * Then from the end, remove the last few points if they are now very improbable.
	 */
	
	
	//Comparator to order poses by likelihood given measurement.
	class PoseComparator<Pose> implements Comparator<Pose>{
		public double[] meas;
		public cvData data;
		
		public PoseComparator(double[] measurements, cvData dat){
			data = dat;
			meas = measurements;
		}
		@Override
		public int compare(Pose p1, Pose p2) {
			// TODO Auto-generated method stub
			double l1 = data.grid.measLikelihood((comm.BotClientMap.Pose) p1, meas);
			double l2 = data.grid.measLikelihood((comm.BotClientMap.Pose) p2, meas);
			return (int) Math.signum(l1-l2);
		}
		
	}
	
	public TreeSet<Pose> newProspectivePositions(double[] distances){
		//Comparator
		PoseComparator<Pose> pc = new PoseComparator<Pose>(distances, data);
		Pose currentBest = poses.last();
		double x = currentBest.x;
		double y = currentBest.y;
		double theta = currentBest.theta;
		TreeSet<Pose> output = new TreeSet<Pose>(pc);
		TreeSet<Pose> everything = new TreeSet<Pose>(pc);
		for(int xinc = - (int)(xRange/xPrecision); xinc< (int)(xRange/xPrecision);xinc++ ){
			for(int yinc = - (int)(xRange/xPrecision); yinc< (int)(xRange/xPrecision);yinc++ ){
				for(int tinc = - (int)(thetaRange/thetaPrecision); tinc< (int)(thetaRange/thetaPrecision);tinc++ ){
					Pose p = new Pose(x+(xinc*xPrecision), y+(yinc*xPrecision), theta+(tinc*thetaPrecision));
					double likelihood = data.grid.measLikelihood(p, distances);
					if(likelihood > pThresh){
						output.add(p);
					}
					everything.add(p);
				}
			}
		}
		if(output.size()!=0){
			return output;
		}
		else{
			return everything;
		}
	}
	
	public void assessPosition(double[] distances){
		TreeSet<Pose> prosp = newProspectivePositions(distances);
		poses = prosp;
	}
	
	public Pose likelyPos(){
		return poses.last();
	}
	
	
	
	
}
