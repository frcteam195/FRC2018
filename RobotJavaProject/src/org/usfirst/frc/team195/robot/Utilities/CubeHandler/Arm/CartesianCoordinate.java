package org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm;

public class CartesianCoordinate {
	public double x;
	public double y;

	public CartesianCoordinate(double x, double y) {
		this.x = x;
		this.y = y;
	}

	private PolarCoordinate getPolar() {
		double r = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
		double theta = Math.toDegrees(Math.atan2(y, x));
		return new PolarCoordinate(r, theta);
	}
}
