package org.usfirst.frc.team195.robot.Actions;

import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystemOld;
import org.usfirst.frc.team195.robot.Utilities.CustomAction;

public class ShiftAction extends CustomAction {
	private DriveBaseSubsystemOld driveBaseSubsystem;
	
	public ShiftAction() {
		super();
		driveBaseSubsystem = DriveBaseSubsystemOld.getInstance();
	}

	@Override
	public void start() {
		;
	}
	
	public void start(boolean highGear) {
		driveBaseSubsystem.setGear(highGear);
	}
	
	@Override
	public void run() {
		;
	}

}