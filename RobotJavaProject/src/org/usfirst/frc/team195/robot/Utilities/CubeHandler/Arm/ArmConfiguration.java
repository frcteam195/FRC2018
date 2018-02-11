package org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm;

import org.usfirst.frc.team195.robot.Utilities.Constants;

public class ArmConfiguration {
	public static final PolarCoordinate HOME = new PolarCoordinate(-1, -1);
	public static final PolarCoordinate STRAIGHT = new PolarCoordinate((Constants.kArm1Length + Constants.kArm2Length), 90);

	private double a1Angle;
	private double a2Angle;

	public ArmConfiguration(double a1Angle, double a2Angle) {
		this.a1Angle = a1Angle;
		this.a2Angle = a2Angle;
	}

	public double getA1Angle() {
		return a1Angle;
	}

	public double getA2Angle() {
		return a2Angle;
	}

	@Override
	public String toString() {
		return "Angle1: " + getA1Angle() + ", Angle2: " + getA2Angle();
	}
}
