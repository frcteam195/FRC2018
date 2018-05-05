package com.team195.utils;

public class Util {
	public static double max(double x, double y) {
		if (x > y)
			return x;
		else
			return y;
	}

	public static double absMax(double x, double y) {
		return max(Math.abs(x), Math.abs(y));
	}

	public static double[] normalizeDriveValues(double left, double right, double maxOuput) {
		double[] normal = new double[2];

		double maxValue = absMax(left, right);

		normal[0] = maxValue > maxOuput ? left / maxValue : left;
		normal[1] = maxValue > maxOuput ? right / maxValue : right;

		return normal;
	}
}
