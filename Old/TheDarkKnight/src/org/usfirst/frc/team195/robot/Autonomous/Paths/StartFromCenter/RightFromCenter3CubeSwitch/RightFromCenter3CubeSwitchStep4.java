package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromCenter.RightFromCenter3CubeSwitch;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightFromCenter3CubeSwitchStep4 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(120,106,0,0));
		sWaypoints.add(new Waypoint(60,105,40,26,"DropArm"));
		sWaypoints.add(new Waypoint(96,138,0,26));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(120, 106), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}