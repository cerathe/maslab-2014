package edu.mit.felixsun.maslab;

public class UnStuckState {
	int turnTime = 10;
	int backTime = 30;
	double backSpeed = -5;
	double turnSpeed = -1;
	int counter;
	int substate;
	
	Sensors s;
	Localization l;
	
	DriveStraightState backUp = new DriveStraightState();
	TurnState turn = new TurnState();
	
	public UnStuckState(Localization loc){
		l = loc;
		substate=1;
		counter = backTime;
	}
	
	public int step(Sensors s){
		if(substate ==1){ //back up
			s.leftDriveMotor.setSpeed(-Constants.SPEED);
			s.rightDriveMotor.setSpeed(Constants.SPEED);
			if(counter>0){
				counter--;
			}
			else{
				counter = turnTime;
				substate = 2;
			}
			return 0;
		}
		else if(substate ==2){ //turn around.
			turn.step(l,s, turnSpeed);
			if(counter>1){
				counter--;
				return 0;
			}
			else if(counter ==1){
				counter --;
				l.relocalize = true;
				return 2;
			}
			else{
				counter = backTime;
				substate = 1;
				s.rightDriveMotor.setSpeed(0);
				s.leftDriveMotor.setSpeed(0);
			}
		}
		return 1;
	}
	
}
