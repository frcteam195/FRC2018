package org.usfirst.frc.team195.robot.Autonomous.Paths.Fields;

import edu.wpi.first.wpilibj.DriverStation;

public class PracticeField implements FieldProfile {
	@Override
	public double getSwitchX() {
		if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue)
			return 197.5;
		else
			return 197.5;
	}

	@Override
	public double getScaleX() {
		if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue)
			return 297.5;
		else
			return 297.5;
	}
}
