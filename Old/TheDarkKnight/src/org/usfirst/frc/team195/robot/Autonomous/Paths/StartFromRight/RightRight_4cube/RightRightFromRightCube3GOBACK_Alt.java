package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_4cube;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightRightFromRightCube3GOBACK_Alt implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(234,113,0,0));
		sWaypoints.add(new Waypoint(270,89,10,50, "PreparePlaceCube"));
		sWaypoints.add(PathAdapter.getAdaptedRightScaleWaypoint(new Waypoint(286,92,0,50)));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(238, 111), Rotation2d.fromDegrees(180));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}