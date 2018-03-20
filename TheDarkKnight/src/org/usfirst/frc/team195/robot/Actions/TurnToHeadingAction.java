package org.usfirst.frc.team195.robot.Actions;

import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Path;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathContainer;

/**
 * Drives the robot along the Path defined in the PathContainer object. The action finishes once the robot reaches the
 * end of the path.
 *
 */
public class TurnToHeadingAction implements Action {

	private DriveBaseSubsystem mDrive = DriveBaseSubsystem.getInstance();
	private double heading;
	private boolean headingIsAbsolute = false;

	public TurnToHeadingAction(double rotationDeg, boolean headingIsAbsolute) {
		heading = rotationDeg;
		this.headingIsAbsolute = headingIsAbsolute;
	}

	@Override
	public boolean isFinished() {
		return mDrive.isDoneWithTurn();
	}

	@Override
	public void update() {
		// Nothing done here, controller updates in mEnabedLooper in robot
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mDrive.setTurnToHeading(heading, headingIsAbsolute);
	}
}
