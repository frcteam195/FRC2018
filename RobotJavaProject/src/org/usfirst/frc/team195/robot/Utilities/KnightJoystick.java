package org.usfirst.frc.team195.robot.Utilities;

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
	
	public boolean GetRisingEdgeButton(int button) {
		try {
			boolean currentButton = super.getRawButton(button);
			boolean retVal = (currentButton != prevButtonVal[button-1]) && currentButton;
			prevButtonVal[button-1] = currentButton;
			return retVal;
		} catch(Exception ex) {
			return false;
		}
	}
	
	public boolean GetFallingEdgeButton(int button) {
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