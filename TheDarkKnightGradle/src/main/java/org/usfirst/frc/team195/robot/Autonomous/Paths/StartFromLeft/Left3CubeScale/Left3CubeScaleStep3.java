package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.Left3CubeScale;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class Left3CubeScaleStep3 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(PathAdapter.getAdaptedLeftSwitchWaypoint(new Waypoint(230,241,0,0)));
		sWaypoints.add(new Waypoint(248,252,15,50, "PreparePlaceCube"));
		sWaypoints.add(PathAdapter.getAdaptedLeftScaleWaypoint(new Waypoint(278,244,0,50)));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(230, 241), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}