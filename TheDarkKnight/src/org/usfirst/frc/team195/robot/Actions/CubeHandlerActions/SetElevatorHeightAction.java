package org.usfirst.frc.team195.robot.Actions.CubeHandlerActions;

import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;

public class SetElevatorHeightAction implements Action {
	private double elevatorHeight = 0;
	private CubeHandlerSubsystem mCubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();

	public SetElevatorHeightAction(double elevatorHeight) {
		this.elevatorHeight = elevatorHeight;
	}

	@Override
	public boolean isFinished() {
		return mCubeHandlerSubsystem.isElevatorAtSetpoint();
	}

	@Override
	public void update() {
		//
	}

	@Override
	public void done() {
		//
	}

	@Override
	public void start() {
		mCubeHandlerSubsystem.setElevatorHeight(elevatorHeight);
		mCubeHandlerSubsystem.setBlinkOnHome(false);
	}
}
