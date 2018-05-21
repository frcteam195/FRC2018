package org.usfirst.frc.team195.robot.Actions.CubeHandlerActions;

import org.usfirst.frc.team195.robot.Actions.Framework.Action;
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
		return mCubeHandlerSubsystem.isElevatorAtSetpoint();
	}

	@Override
	public void update() {
		// Add this statement in update to continually set the height to fix bug with a slow homing elevator where
		// sometimes homing wouldn't finish fast enough and the elevator would not raise up
		//MAY INTRODUCE A BUG FOR CONCURRENT TELEOP SETS - come up with something better => (check if auto?)
		//TODO: Test this auto fix
		if (mCubeHandlerSubsystem.isAuto() && (updateFrequencyDividerCounter++ % kUpdateFrequencyDivisor) == 0)
			mCubeHandlerSubsystem.setElevatorHeight(elevatorHeight);
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
