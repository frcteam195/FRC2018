package org.usfirst.frc.team195.robot.Actions.CubeHandlerActions;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;

public class SetIntakeClampAction implements Action {
	private boolean intakeOpen;
	private CubeHandlerSubsystem mCubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();
	private double startTime = 0;
	private double waitTime;

	public SetIntakeClampAction(boolean intakeOpen) {
		this.intakeOpen = intakeOpen;
		this.waitTime = 0.1;
	}

	public SetIntakeClampAction(boolean intakeOpen, double waitTime) {
		this.intakeOpen = intakeOpen;
		this.waitTime = waitTime;
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - startTime > waitTime;
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
		startTime = Timer.getFPGATimestamp();
		mCubeHandlerSubsystem.setIntakeClamp(intakeOpen);
	}
}
