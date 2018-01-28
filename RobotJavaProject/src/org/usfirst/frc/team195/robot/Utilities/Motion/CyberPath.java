package org.usfirst.frc.team195.robot.Utilities.Motion;

import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.Motion.SRX.SRXDriveBaseTrajectory;
import org.usfirst.frc.team195.robot.Utilities.Motion.SRX.SRXTrajectoryConfig;

public class CyberPath {
	private SRXTrajectoryConfig srxTrajectoryConfig;
	private WaypointSequence waypointSequence;

	public CyberPath() {
		this.srxTrajectoryConfig = Constants.SXC;
		waypointSequence = new WaypointSequence();
	}

	public CyberPath(SRXTrajectoryConfig srxTrajectoryConfig) {
		this.srxTrajectoryConfig = srxTrajectoryConfig;
		waypointSequence = new WaypointSequence();
	}

	/**
	 * Add a point to the CyberPath
	 * @param x X Coordinate
	 * @param y Y Coordinate
	 * @param theta Heading in degrees
	 */
	public void addPoint(double x, double y, double theta) {
		waypointSequence.addWaypoint(new WaypointSequence.Waypoint(x, y, Math.toRadians(theta)));
	}

	public SRXDriveBaseTrajectory getSRXTrajectory() {
		Path p = PathGenerator.makePath(waypointSequence, srxTrajectoryConfig, srxTrajectoryConfig.wheelbaseWidthInches, srxTrajectoryConfig.name);
		return new SRXDriveBaseTrajectory(getSRXPoints(p.getPair().left), getSRXPoints(p.getPair().right));
	}

	private double[][] getSRXPoints(Trajectory trajectory) {
		double[][] points = new double[trajectory.getNumSegments()][3];

		for (int i = 0; i < trajectory.getNumSegments(); i++) {
			points[i][0] = convertInchesToEncoderRotations(trajectory.getSegment(i).pos, srxTrajectoryConfig.wheelDiameterInches, srxTrajectoryConfig.encoderRotToWheelRotFactor);
			points[i][1] = convertInchesPerSecondToEncoderTicksPer100ms(trajectory.getSegment(i).vel, srxTrajectoryConfig.wheelDiameterInches, srxTrajectoryConfig.encoderRotToWheelRotFactor, srxTrajectoryConfig.encoderTicksPerRev);
			points[i][2] = trajectory.getSegment(i).dt * 1000;
		}
		return points;
	}

	private double convertInchesToEncoderRotations(double inches, double wheelDiameterInches, double scaleFactor) {
		return (inches / (wheelDiameterInches * Math.PI)) * scaleFactor;
	}

	private double convertInchesPerSecondToEncoderTicksPer100ms(double ips, double wheelDiameterInches, double scaleFactor, int encoderTicksPerRev) {
		return (ips * 60) / (wheelDiameterInches * Math.PI) * scaleFactor * encoderTicksPerRev / 600;
	}

}
