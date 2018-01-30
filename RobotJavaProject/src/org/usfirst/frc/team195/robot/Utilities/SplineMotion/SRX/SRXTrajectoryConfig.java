package org.usfirst.frc.team195.robot.Utilities.SplineMotion.SRX;

import org.usfirst.frc.team195.robot.Utilities.SplineMotion.TrajectoryGenerator;

public class SRXTrajectoryConfig extends TrajectoryGenerator.Config {
	public String name;
	public double wheelbaseWidthInches;
	public double wheelDiameterInches;
	public int encoderTicksPerRev;
	public double encoderRotToWheelRotFactor;
	public boolean reversed = false;
}
