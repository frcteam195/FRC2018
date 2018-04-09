package org.usfirst.frc.team195.robot.Autonomous.Paths.Robots;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Translation2d;

public class PracticeBot extends RobotProfile {
	@Override
	public Translation2d getTransform() {
		return new Translation2d(0, 0);
	}

	@Override
	public final double getWheelDiameter() {
		return 4.875;
	}
}
