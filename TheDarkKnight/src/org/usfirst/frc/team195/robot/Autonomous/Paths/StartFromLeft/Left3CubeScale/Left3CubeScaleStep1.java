package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.Left3CubeScale;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class Left3CubeScaleStep1 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,276,0,0));
		sWaypoints.add(new Waypoint(160,276,30,130,"PreparePlaceCube"));
		sWaypoints.add(new Waypoint(233,264,30,130));
		sWaypoints.add(PathAdapter.getAdaptedLeftScaleWaypoint(new Waypoint(278,248,0,100)));

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