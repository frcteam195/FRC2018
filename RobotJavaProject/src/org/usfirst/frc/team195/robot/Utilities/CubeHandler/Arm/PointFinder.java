package org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm;

import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

import java.util.ArrayList;

public class PointFinder {
	private double minA1Angle = Constants.kArm1SoftMin * 360;
	private double maxA1Angle = Constants.kArm1SoftMax * 360;
	private double minA2Angle = Constants.kArm2SoftMin * 360;
	private double maxA2Angle = Constants.kArm2SoftMax * 360;
	private double kA1Length = Constants.kArm1Length;
	private double kA2Length = Constants.kArm2Length;

	private ArrayList<PresetPoint> presetPoints;

	public PointFinder() {
		presetPoints = new ArrayList<PresetPoint>();
		presetPoints.add(new PresetPoint(ArmConfiguration.HOME, new ArmConfiguration(183, -165)));
	}

	public ArmConfiguration getArmConfigFromPolar(PolarCoordinate polarCoordinate) {
		for (PresetPoint p : presetPoints) {
			if (p.polarCoordinate.compareTo(polarCoordinate) == 0)
				return p.armConfiguration;
		}

		CartesianCoordinate cc = polarCoordinate.getCartesian();
		double distance = polarCoordinate.r;
		distance = distance > kA1Length + kA2Length ? kA1Length + kA2Length : distance;
		double d1 = polarCoordinate.theta;
		d1 = d1 > 180 ? 180 : d1;
		d1 = d1 < 0 ? 0 : d1;
		double d2 = Math.toDegrees(Math.acos((Math.pow(distance,2) + Math.pow(kA1Length,2) - Math.pow(kA2Length,2))
				/(2*distance*kA1Length)));
		double phi = cc.x < 0 ? d1 - d2 : d1 + d2;
		double a2 = Math.toDegrees(Math.acos((Math.pow(kA1Length,2) + Math.pow(kA2Length,2) - + Math.pow(distance,2))
				/(2*kA1Length*kA2Length)));
		double alpha = -(180 - a2) * Math.signum(cc.x);

		if (Double.isNaN(phi) || Double.isNaN(alpha)) {
			ConsoleReporter.report("Arm solution not possible!", MessageLevel.DEFCON1);
			return null;
		}

		phi = Util.limit(phi, minA1Angle, maxA1Angle);
		alpha = Util.limit(alpha, minA2Angle, maxA2Angle);

		return new ArmConfiguration(phi, alpha);
	}

	public void setA1AngleRange(double minA1Angle, double maxA1Angle) {
		this.minA1Angle = minA1Angle;
		this.maxA1Angle = maxA1Angle;
	}

	public void setA2AngleRange(double minA2Angle, double maxA2Angle) {
		this.minA2Angle = minA2Angle;
		this.maxA2Angle = maxA2Angle;
	}
}
