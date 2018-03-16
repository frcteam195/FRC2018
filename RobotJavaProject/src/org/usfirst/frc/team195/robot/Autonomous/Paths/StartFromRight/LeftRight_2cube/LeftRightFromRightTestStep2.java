package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftRight_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

//public class LeftRightFromRightTestStep2 implements PathContainer {
//
//	@Override
//	public Path buildPath() {
//		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
//		sWaypoints.add(new Waypoint(252,200,0,0));
//		sWaypoints.add(new Waypoint(248,204,0,30));
//
//		return PathBuilder.buildPathFromWaypoints(sWaypoints);
//	}
//
//	@Override
//	public RigidTransform2d getStartPose() {
//		return new RigidTransform2d(new Translation2d(252, 200), Rotation2d.fromDegrees(0));
//	}
//
//	@Override
//	public boolean isReversed() {
//		return false;
//	}
//	// WAYPOINT_DATA: [{"position":{"x":252,"y":200},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":248,"y":204},"speed":30,"radius":0,"marker":"","comment":""}]
//	// IS_REVERSED: false
//	// FILE_NAME: LeftRightFromRightTestStep2
//}

public class LeftRightFromRightTestStep2 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(252,200,0,0));
		sWaypoints.add(new Waypoint(234,200,0,20));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(252, 200), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA: [{"position":{"x":252,"y":200},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":234,"y":200},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftRightFromRightTestStep2
}