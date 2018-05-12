package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromCenter.Right3Cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightFromCenterStep3 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(50,163,0,0));
		sWaypoints.add(new Waypoint(82,163,0,40));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(50, 163), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA: [{"position":{"x":50,"y":163},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":82,"y":163},"speed":40,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightFromCenterStep3
}