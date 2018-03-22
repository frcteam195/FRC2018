package org.usfirst.frc.team195.robot.Utilities.Drivers;

import edu.wpi.first.wpilibj.DigitalInput;

public class KnightDigitalInput extends DigitalInput {
	private boolean prevInputVal;

	public KnightDigitalInput(int port) {
		super(port);

		prevInputVal = false;
	}

	public synchronized boolean getRisingEdge() {
		try {
			boolean currentInput = super.get();
			boolean retVal = (currentInput != prevInputVal) && currentInput;
			prevInputVal = currentInput;
			return retVal;
		} catch(Exception ex) {
			return false;
		}
	}

	public synchronized boolean getFallingEdge() {
		try {
			boolean currentInput = super.get();
			boolean retVal = (currentInput != prevInputVal) && !currentInput;
			prevInputVal = currentInput;
			return retVal;
		} catch(Exception ex) {
			return false;
		}
	}
}
