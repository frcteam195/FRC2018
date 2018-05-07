package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.Right3CubeScale;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class Right3CubeScaleStep4 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(PathAdapter.getAdaptedRightScaleWaypoint(new Waypoint(282,76,0,0)));
		sWaypoints.add(new Waypoint(247,77,15,40));
		sWaypoints.add(PathAdapter.getAdaptedRightSwitchWaypoint(new Waypoint(228,98,0,40)));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(282, 76), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}