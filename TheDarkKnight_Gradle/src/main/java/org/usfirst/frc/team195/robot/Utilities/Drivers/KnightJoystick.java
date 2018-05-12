package org.usfirst.frc.team195.robot.Utilities.Drivers;

import edu.wpi.first.wpilibj.Joystick;

public class KnightJoystick extends Joystick {
	private boolean[] prevButtonVal;
	
	public KnightJoystick(int port) {
		super(port);
		
		prevButtonVal = new boolean[16];
		
		for (int i = 0; i < 16; i++) {
			prevButtonVal[i] = false;
		}
	}
	
	public synchronized boolean getRisingEdgeButton(int button) {
		try {
			boolean currentButton = super.getRawButton(button);
			boolean retVal = (currentButton != prevButtonVal[button-1]) && currentButton;
			prevButtonVal[button-1] = currentButton;
			return retVal;
		} catch(Exception ex) {
			return false;
		}
	}
	
	public synchronized boolean getFallingEdgeButton(int button) {
		try {
			boolean currentButton = super.getRawButton(button);
			boolean retVal = (currentButton != prevButtonVal[button-1]) && !currentButton;
			prevButtonVal[button-1] = currentButton;
			return retVal;
		} catch(Exception ex) {
			return false;
		}
	}

}