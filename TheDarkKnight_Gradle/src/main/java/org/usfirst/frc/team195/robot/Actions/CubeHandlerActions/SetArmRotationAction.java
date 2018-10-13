package org.usfirst.frc.team195.robot.Actions.CubeHandlerActions;

import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.AutoRequestUpdater;
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
		ConsoleReporter.report("Desired Arm Position: " + CubeHandlerSubsystem.getInstance().getArmDesiredPosition());
		ConsoleReporter.report("Actual Arm Position: " + CubeHandlerSubsystem.getInstance().getArmRotationDeg());

		return mCubeHandlerSubsystem.isArmAtSetpoint();
		//return mCubeHandlerSubsystem.getArmRotationDeg() < armRotationDeg;
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
		mCubeHandlerSubsystem.setArmRotationDeg(armRotationDeg);
	}
}
