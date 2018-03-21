package org.usfirst.frc.team195.robot.Autonomous.Paths.Fields;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Translation2d;

public class CompetitionTheoreticalField implements FieldProfile {

	@Override
	public double getSwitchX() {
		if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue)
			return 196;
		else
			return 196;
	}

	@Override
	public double getScaleX() {
		if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue)
			return 300;
		else
			return 300;
	}

}
