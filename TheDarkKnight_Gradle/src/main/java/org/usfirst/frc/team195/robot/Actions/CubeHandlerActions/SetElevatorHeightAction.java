package org.usfirst.frc.team195.robot.Actions.CubeHandlerActions;

import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.AutoRequestUpdater;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;

public class SetElevatorHeightAction implements Action {
	private double elevatorHeight = 0;
	private CubeHandlerSubsystem mCubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();

	private int updateFrequencyDividerCounter = 0;
	private static final int kUpdateFrequencyDivisor = 5;

	public SetElevatorHeightAction(double elevatorHeight) {
		this.elevatorHeight = elevatorHeight;
	}

	@Override
	public boolean isFinished() {
		ConsoleReporter.report("Desired Elevator Position: " + CubeHandlerSubsystem.getInstance().getElevatorDesiredHeight());
		ConsoleReporter.report("Actual Elevator Position: " + CubeHandlerSubsystem.getInstance().getElevatorHeight());
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
