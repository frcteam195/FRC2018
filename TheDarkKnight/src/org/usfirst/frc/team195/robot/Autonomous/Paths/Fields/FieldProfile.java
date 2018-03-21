package org.usfirst.frc.team195.robot.Autonomous.Paths.Fields;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Translation2d;

public interface FieldProfile {
	/**
	 * The back corner of the switch
	 * @return X-Coord to the back corner of the switch in inches
	 */
	double getSwitchX();

	/**
	 * The front corner of the switch
	 * @return X-Coord to the front corner of the switch in inches
	 */
	double getScaleX();
}
