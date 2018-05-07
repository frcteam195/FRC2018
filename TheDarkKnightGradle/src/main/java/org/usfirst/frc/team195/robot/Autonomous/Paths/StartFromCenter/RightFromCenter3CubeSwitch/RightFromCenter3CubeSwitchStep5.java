package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromCenter.RightFromCenter3CubeSwitch;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightFromCenter3CubeSwitchStep5 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
		sWaypoints.add(new PathBuilder.Waypoint(96,138,0,0));
		sWaypoints.add(new PathBuilder.Waypoint(60,105,40,35));
		sWaypoints.add(new PathBuilder.Waypoint(120,106,0,35));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(86, 140), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}