package org.usfirst.frc.team195.robot.Actions;

import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team195.robot.Utilities.CustomAction;

public class ShiftAction extends CustomAction {
	private DriveBaseSubsystem driveBaseSubsystem;
	
	public ShiftAction() {
		super();
		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
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