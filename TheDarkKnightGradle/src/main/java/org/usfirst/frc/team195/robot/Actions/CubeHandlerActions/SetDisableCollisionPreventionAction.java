package org.usfirst.frc.team195.robot.Actions.CubeHandlerActions;

import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;

public class SetDisableCollisionPreventionAction implements Action {
	private boolean disable;
	private CubeHandlerSubsystem mCubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();

	public SetDisableCollisionPreventionAction(boolean disable) {
		this.disable = disable;
	}

	@Override
	public boolean isFinished() {
		return mCubeHandlerSubsystem.isCollisionPreventionDisabled() == disable;
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
		ConsoleReporter.report("Disable Collision Action! : " + disable);
		mCubeHandlerSubsystem.setDisableCollisionPrevention(disable);
	}
}