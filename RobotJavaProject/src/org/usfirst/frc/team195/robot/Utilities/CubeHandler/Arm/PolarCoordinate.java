package org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm;

public class PolarCoordinate implements Comparable<PolarCoordinate> {
	public double r;
	public double theta;

	public PolarCoordinate(double r, double theta) {
		this.r = r;
		this.theta = theta;
	}

	@Override
	public int compareTo(PolarCoordinate o) {
		if (o.r == r && o.theta == theta)
			return 0;
		return -1;
	}

	public CartesianCoordinate getCartesian() {
		double x = r*Math.cos(Math.toRadians(theta));
		double y = r*Math.sin(Math.toRadians(theta));
		return new CartesianCoordinate(x, y);
	}
}
