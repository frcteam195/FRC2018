package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_4cube;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightRightFromRightStep2 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(PathAdapter.getAdaptedRightScaleWaypoint(new Waypoint(286,88,0,0)));
		sWaypoints.add(new Waypoint(257,82,15,40));
		sWaypoints.add(PathAdapter.getAdaptedRightSwitchWaypoint(new Waypoint(233,89,0,30)));

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
}