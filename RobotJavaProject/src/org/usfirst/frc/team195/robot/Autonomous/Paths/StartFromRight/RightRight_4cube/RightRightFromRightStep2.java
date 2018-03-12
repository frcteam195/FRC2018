package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_4cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightRightFromRightStep2 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(286,88,0,0));
		sWaypoints.add(new Waypoint(257,82,15,60));
		sWaypoints.add(new Waypoint(231,90,0,30));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(286, 88), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA: [{"position":{"x":286,"y":86},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":257,"y":82},"speed":20,"radius":15,"marker":"","comment":""},{"position":{"x":228,"y":90},"speed":20,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightRightFromRightStep2
}