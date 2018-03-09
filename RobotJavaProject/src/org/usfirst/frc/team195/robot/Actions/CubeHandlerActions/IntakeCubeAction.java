package org.usfirst.frc.team195.robot.Actions.CubeHandlerActions;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;

public class IntakeCubeAction implements Action {
	private CubeHandlerSubsystem mCubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();
	private double startTime = 0;
	private double timeout = 0;

	/**
	 * An action to intake a cube until the cube sensor trips
	 * @param timeout Timeout in seconds. Value of 0 will disable timeout
	 */
	public IntakeCubeAction(double timeout) {
		this.timeout = timeout;
	}

	@Override
	public boolean isFinished() {
		if (timeout > 0)
			return mCubeHandlerSubsystem.hasCube() || (Timer.getFPGATimestamp() - startTime) > timeout;
		else
			return mCubeHandlerSubsystem.hasCube();
	}

	@Override
	public void update() {
		//
	}

	@Override
	public void done() {
		mCubeHandlerSubsystem.setIntakeControl(IntakeControl.HOLD);
	}

	@Override
	public void start() {
		startTime = Timer.getFPGATimestamp();
		mCubeHandlerSubsystem.setIntakeControl(IntakeControl.INTAKE_IN);
	}
}
