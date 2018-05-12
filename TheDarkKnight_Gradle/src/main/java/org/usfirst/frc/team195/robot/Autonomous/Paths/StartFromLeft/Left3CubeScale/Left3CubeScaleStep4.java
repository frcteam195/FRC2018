package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.Left3CubeScale;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class Left3CubeScaleStep4 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(PathAdapter.getAdaptedLeftScaleWaypoint(new Waypoint(278,244,0,0)));
		sWaypoints.add(new Waypoint(252,248,15,45));
		sWaypoints.add(PathAdapter.getAdaptedLeftSwitchWaypoint(new Waypoint(224,216,0,45)));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(278, 244), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}