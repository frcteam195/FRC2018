package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.Right3CubeScale;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class Right3CubeScaleStep1 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,276,0,0));
		sWaypoints.add(new Waypoint(110,276,30,130));
		sWaypoints.add(new Waypoint(220,282,30,130));
		sWaypoints.add(new Waypoint(242,214,15,130));
		sWaypoints.add(new Waypoint(242,120,0,100,"PreparePlaceCube"));
		sWaypoints.add(new Waypoint(242,76,15,80));
		sWaypoints.add(PathAdapter.getAdaptedRightScaleWaypoint(new Waypoint(280,74,0,60)));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(20, 276), Rotation2d.fromDegrees(180));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}