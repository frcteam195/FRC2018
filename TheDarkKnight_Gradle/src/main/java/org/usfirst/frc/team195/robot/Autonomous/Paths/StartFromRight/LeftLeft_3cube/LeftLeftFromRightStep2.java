package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftLeft_3cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftLeftFromRightStep2 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(290,257,0,0));
		sWaypoints.add(new Waypoint(253,257,15,50,"PreparePickupCube"));
		sWaypoints.add(new Waypoint(217,239,0,30));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(290, 257), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA: [{"position":{"x":290,"y":257},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":253,"y":257},"speed":50,"radius":15,"marker":"PreparePickupCube","comment":""},{"position":{"x":217,"y":239},"speed":30,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftLeftFromRightStep2
}