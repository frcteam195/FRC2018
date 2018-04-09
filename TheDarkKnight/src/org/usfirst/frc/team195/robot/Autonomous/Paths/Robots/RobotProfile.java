package org.usfirst.frc.team195.robot.Autonomous.Paths.Robots;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Translation2d;

/**
 * Y values with respect to the left side of the field, so a minus value will move the robot farther right
 */
public abstract class RobotProfile {
	public abstract Translation2d getTransform();

	/**
	 * Used to account for wheel wear over time.
	 * @return Wheel diameter of the robot, accounting for wear.
	 * Value calculation is (Measured value of current bot - Measured value of practice bot) + 4.875
	 */
	public abstract double getWheelDiameter();
}
