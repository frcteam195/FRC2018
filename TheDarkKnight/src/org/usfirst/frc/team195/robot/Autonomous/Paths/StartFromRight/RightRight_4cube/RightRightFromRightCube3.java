package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_4cube;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

//public class RightRightFromRightCube3 implements PathContainer {
//
//	@Override
//	public Path buildPath() {
//		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
//		sWaypoints.add(PathAdapter.getAdaptedRightScaleWaypoint(new Waypoint(286,89,0,20)));
//		sWaypoints.add(new Waypoint(260,75,15,50));
//		sWaypoints.add(new Waypoint(227,108,0,50));
//
//		return PathBuilder.buildPathFromWaypoints(sWaypoints);
//	}
//
//	@Override
//	public RigidTransform2d getStartPose() {
//		return new RigidTransform2d(new Translation2d(286, 89), Rotation2d.fromDegrees(0));
//	}
//
//	@Override
//	public boolean isReversed() {
//		return false;∂∂∂
//	}
//}

public class RightRightFromRightCube3 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(PathAdapter.getAdaptedRightScaleWaypoint(new Waypoint(286,89,0,20)));
		sWaypoints.add(new Waypoint(260,75,15,50));
		sWaypoints.add(new Waypoint(245,110,10,40));
		sWaypoints.add(new Waypoint(233,114,0,30));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(286, 89), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}