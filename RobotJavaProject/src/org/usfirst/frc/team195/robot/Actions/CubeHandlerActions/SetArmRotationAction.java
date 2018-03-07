package org.usfirst.frc.team195.robot.Actions.CubeHandlerActions;

import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;

public class SetArmRotationAction implements Action {
	private double armRotationDeg = 0;
	private CubeHandlerSubsystem mCubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();

	public SetArmRotationAction(double armRotationDeg) {
		this.armRotationDeg = armRotationDeg;
	}

	@Override
	public boolean isFinished() {
		return mCubeHandlerSubsystem.isArmAtSetpoint();
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
