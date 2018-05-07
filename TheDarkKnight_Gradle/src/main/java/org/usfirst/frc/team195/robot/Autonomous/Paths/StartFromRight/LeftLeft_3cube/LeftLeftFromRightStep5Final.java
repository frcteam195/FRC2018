package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftLeft_3cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftLeftFromRightStep5Final implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(220,210,0,0));
		sWaypoints.add(new Waypoint(243,216,15,80));
		sWaypoints.add(new Waypoint(247,251,15,60));
		sWaypoints.add(new Waypoint(288,248,0,50));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(220, 210), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
	// WAYPOINT_DATA: [{"position":{"x":220,"y":210},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":243,"y":216},"speed":100,"radius":15,"marker":"","comment":""},{"position":{"x":247,"y":251},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":288,"y":243},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: LeftLeftFromRightStep5Final
}