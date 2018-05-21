package org.usfirst.frc.team195.robot.Actions.CubeHandlerActions;

import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;

public class SetArmRotationAction implements Action {
	private double armRotationDeg = 0;
	private CubeHandlerSubsystem mCubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();
	private int updateFrequencyDividerCounter = 0;
	private static final int kUpdateFrequencyDivisor = 5;

	public SetArmRotationAction(double armRotationDeg) {
		this.armRotationDeg = armRotationDeg;
	}

	@Override
	public boolean isFinished() {
		return mCubeHandlerSubsystem.isArmAtSetpoint();
		//return mCubeHandlerSubsystem.getArmRotationDeg() < armRotationDeg;
	}

	@Override
	public void update() {
		// Add this statement in update to continually set the rotation to fix bug with a resetting arm where
		// sometimes reset wouldn't finish fast enough and the arm would not move
		//MAY INTRODUCE A BUG FOR CONCURRENT TELEOP SETS - come up with something better => (check if auto?)
		//TODO: Test this auto fix
		if (mCubeHandlerSubsystem.isAuto() && (updateFrequencyDividerCounter++ % kUpdateFrequencyDivisor) == 0)
			mCubeHandlerSubsystem.setArmRotationDeg(armRotationDeg);
	}

	@Override
	public void done() {
		//
	}

	@Override
	public void start() {
		mCubeHandlerSubsystem.setArmRotationDeg(armRotationDeg);
	}
}
