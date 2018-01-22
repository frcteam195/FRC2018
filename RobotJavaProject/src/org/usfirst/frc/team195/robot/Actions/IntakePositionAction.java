package org.usfirst.frc.team195.robot.Actions;

import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;
import org.usfirst.frc.team195.robot.Utilities.CustomAction;

public class IntakePositionAction extends CustomAction {
	private CubeHandlerSubsystem cubeHandlerSubsystem;
	
	public IntakePositionAction() {
		super();
		cubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();
	}

	@Override
	public void start() {
		;
	}
	
	public void start(boolean open) {
		cubeHandlerSubsystem.setIntakeOpen(open);
	}
	
	@Override
	public void run() {
		;
	}

}
