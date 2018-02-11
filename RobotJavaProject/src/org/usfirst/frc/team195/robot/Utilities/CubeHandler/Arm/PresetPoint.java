package org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm;

public class PresetPoint {
	public PolarCoordinate polarCoordinate;
	public ArmConfiguration armConfiguration;
	public PresetPoint(PolarCoordinate polarCoordinate, ArmConfiguration armConfiguration) {
		this.polarCoordinate = polarCoordinate;
		this.armConfiguration = armConfiguration;
	}
}
