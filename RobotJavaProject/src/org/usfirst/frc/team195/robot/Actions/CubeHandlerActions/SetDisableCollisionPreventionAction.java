package org.usfirst.frc.team195.robot.Actions.CubeHandlerActions;

import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;

public class SetDisableCollisionPreventionAction implements Action {
	private boolean disable = false;
	private CubeHandlerSubsystem mCubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();

	public SetDisableCollisionPreventionAction(boolean disable) {
		this.disable = disable;
	}

	@Override
	public boolean isFinished() {
		return true;
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
		mCubeHandlerSubsystem.setDisableCollisionPrevention(disable);
	}
}