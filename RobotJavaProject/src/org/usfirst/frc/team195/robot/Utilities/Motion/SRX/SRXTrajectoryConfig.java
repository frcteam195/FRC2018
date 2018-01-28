package org.usfirst.frc.team195.robot.Utilities.Motion.SRX;

import org.usfirst.frc.team195.robot.Utilities.Motion.TrajectoryGenerator;

public class SRXTrajectoryConfig extends TrajectoryGenerator.Config {
	public String name;
	public double wheelbaseWidthInches;
	public double wheelDiameterInches;
	public int encoderTicksPerRev;
	public double encoderRotToWheelRotFactor;
	public boolean reversed = false;
}
