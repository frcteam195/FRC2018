package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_4cube;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

//public class RightRightFromRightStep5 implements PathContainer {
//
//	@Override
//	public Path buildPath() {
//		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
//		sWaypoints.add(new Waypoint(225,66,0,0));
//		sWaypoints.add(new Waypoint(248,104,15,15));
//		sWaypoints.add(new Waypoint(230,108,0,10));
//
//		return PathBuilder.buildPathFromWaypoints(sWaypoints);
//	}
//
//	@Override
//	public RigidTransform2d getStartPose() {
//		return new RigidTransform2d(new Translation2d(225, 66), Rotation2d.fromDegrees(0));
//	}
//
//	@Override
//	public boolean isReversed() {
//		return false;
//	}
//	// WAYPOINT_DATA: [{"position":{"x":225,"y":66},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":248,"y":104},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":230,"y":108},"speed":60,"radius":0,"marker":"","comment":""}]
//	// IS_REVERSED: false
//	// FILE_NAME: RightRightFromRightStep5
//}

public class RightRightFromRightStep5 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(225,66,0,0));
		sWaypoints.add(new Waypoint(232,80,10,40));
		sWaypoints.add(PathAdapter.getAdaptedRightSwitchWaypoint(new Waypoint(220,111,0,40)));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(225, 66), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA: [{"position":{"x":225,"y":66},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":232,"y":80},"speed":40,"radius":10,"marker":"","comment":""},{"position":{"x":220,"y":111},"speed":40,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightRightFromRightStep5
}