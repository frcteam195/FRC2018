package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightLeft_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightLeftFromRightStep3 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(227,242,0,0));
		sWaypoints.add(new Waypoint(245,245,12,30,"PreparePlaceCube"));
		sWaypoints.add(new Waypoint(269,253,12,30));
		sWaypoints.add(new Waypoint(285,246,0,30));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(227, 242), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
	// WAYPOINT_DATA: [{"position":{"x":227,"y":242},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":245,"y":245},"speed":30,"radius":12,"marker":"PreparePlaceCube","comment":""},{"position":{"x":269,"y":253},"speed":30,"radius":12,"marker":"","comment":""},{"position":{"x":285,"y":246},"speed":30,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: RightLeftFromRightStep3
}