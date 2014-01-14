package edu.mit.felixsun.maslab;

public class State {
	/*
	 * A recursive state machine framework for robot control.
	 * Each state defines init and step functions.  The step function should
	 * return Maple commands, or 0='Success' or 1='Failure' single bytes.  (For now.  This
	 * is still very undecided.)
	 */
	
	public State() {
	}
	
	public void step(cvData data) {
	}
}