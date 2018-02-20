package org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm;

import org.usfirst.frc.team195.robot.Utilities.Constants;

public class ArmConfiguration implements Comparable<ArmConfiguration> {
	public static final PolarCoordinate HOME = new PolarCoordinate(-1, -1);
	public static final PolarCoordinate STRAIGHT = new PolarCoordinate((Constants.kArm1Length + Constants.kArm2Length), 90);
	public static final PolarCoordinate LEFT = new PolarCoordinate((Constants.kArm1Length + Constants.kArm2Length), 180);
	public static final PolarCoordinate RIGHT = new PolarCoordinate((Constants.kArm1Length + Constants.kArm2Length), 0);

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

	public PolarCoordinate getPolarFromAngles() {
		double a = Constants.kArm1Length;
		double b = Constants.kArm2Length;
		double r = Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2) - 2 * a * b * Math.cos(Math.toRadians(180.0-a2Angle)));
		double theta = Math.toDegrees(Math.acos((a*Math.cos(Math.toRadians(a1Angle))+b*Math.sin(Math.toRadians(90-a2Angle+a1Angle)))/r));

		if (Double.isNaN(r) || Double.isNaN(theta))
			return new PolarCoordinate(0, 0);

		return new PolarCoordinate(r, theta);
	}

	@Override
	public String toString() {
		return "Angle1: " + a1Angle + ", Angle2: " + a2Angle;
	}

	@Override
	public int compareTo(ArmConfiguration o) {
		if (o.getA1Angle() == a1Angle && o.getA2Angle() == a2Angle)
			return 0;
		return -1;
	}
}
