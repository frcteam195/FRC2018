package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftLeft_3cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftLeftFromRightStep4 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(253,257,0,0));
		sWaypoints.add(new Waypoint(243,222,15,80));
		sWaypoints.add(new Waypoint(220,210,0,60));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(253, 257), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA: [{"position":{"x":253,"y":257},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":243,"y":216},"speed":100,"radius":15,"marker":"","comment":""},{"position":{"x":220,"y":210},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftLeftFromRightStep4
}