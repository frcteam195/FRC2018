package org.usfirst.frc.team195.robot.Actions.CubeHandlerActions;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;

public class SetIntakeAction implements Action {
	private IntakeControl intakeControl = IntakeControl.OFF;
	private CubeHandlerSubsystem mCubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();
	private double startTime = 0;
	private double waitTime;

	public SetIntakeAction(IntakeControl intakeControl) {
		this.intakeControl = intakeControl;
		this.waitTime = 0.01;
	}

	public SetIntakeAction(IntakeControl intakeControl, double waitTime) {
		this.intakeControl = intakeControl;
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
		mCubeHandlerSubsystem.setIntakeControl(intakeControl);
	}
}
