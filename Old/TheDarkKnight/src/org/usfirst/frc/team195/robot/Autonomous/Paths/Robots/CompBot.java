package org.usfirst.frc.team195.robot.Autonomous.Paths.Robots;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Translation2d;

public class CompBot extends RobotProfile {
	@Override
	public final Translation2d getTransform() {
		//return new Translation2d(-2, -3);
		//return new Translation2d(-4, -2);		//Latest testing values
		return new Translation2d(0, 0);
	}

	//(4.92 - 4.789) + 4.875 //Measured on 4/6/18
	@Override
	public final double getWheelDiameter() {
		return 5;
	}
}
