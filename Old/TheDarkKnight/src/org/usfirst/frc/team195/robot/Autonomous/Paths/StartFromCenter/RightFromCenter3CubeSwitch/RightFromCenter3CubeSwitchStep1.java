package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromCenter.RightFromCenter3CubeSwitch;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightFromCenter3CubeSwitchStep1 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,157,0,0));
		sWaypoints.add(new Waypoint(50,157,25,90,"PreparePlaceCube"));
		sWaypoints.add(new Waypoint(80,115,25,80));
		sWaypoints.add(new Waypoint(120,106,0,60));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(20, 157), Rotation2d.fromDegrees(180));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
}