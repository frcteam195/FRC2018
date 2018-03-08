package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_4cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightRightFromRightStep6 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(222,108,0,0));
		sWaypoints.add(new Waypoint(245,70,15,80, "PreparePlaceCube"));
		sWaypoints.add(new Waypoint(286,78,0,60));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(222, 108), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
	// WAYPOINT_DATA: [{"position":{"x":222,"y":108},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":245,"y":70},"speed":80,"radius":15,"marker":"","comment":""},{"position":{"x":286,"y":78},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: RightRightFromRightStep6
}