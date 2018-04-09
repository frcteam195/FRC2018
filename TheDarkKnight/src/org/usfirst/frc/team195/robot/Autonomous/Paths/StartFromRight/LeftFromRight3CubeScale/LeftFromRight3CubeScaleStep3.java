package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftFromRight3CubeScale;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftFromRight3CubeScaleStep3 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(PathAdapter.getAdaptedLeftSwitchWaypoint(new Waypoint(234,253,0,0)));
		sWaypoints.add(new Waypoint(255,258,15,50));
		sWaypoints.add(PathAdapter.getAdaptedLeftScaleWaypoint(new Waypoint(280,252,0,50)));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(233, 253), Rotation2d.fromDegrees(180));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}