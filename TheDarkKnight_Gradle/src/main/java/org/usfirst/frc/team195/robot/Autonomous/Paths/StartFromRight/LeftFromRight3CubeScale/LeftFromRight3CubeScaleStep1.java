package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftFromRight3CubeScale;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftFromRight3CubeScaleStep1 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,48,0,0));
		sWaypoints.add(new Waypoint(110,48,30,130));
		sWaypoints.add(new Waypoint(220,42,30,130));
		sWaypoints.add(new Waypoint(242,110,15,130));
		sWaypoints.add(new Waypoint(242,204,0,100,"PreparePlaceCube"));
		sWaypoints.add(new Waypoint(242,248,15,80));
		sWaypoints.add(PathAdapter.getAdaptedLeftScaleWaypoint(new Waypoint(283,256,0,60)));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(20, 48), Rotation2d.fromDegrees(180));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}