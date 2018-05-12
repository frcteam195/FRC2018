package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_5cubescale;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LaunchinCubes implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,48,0,0));
		sWaypoints.add(new Waypoint(60,48,0,137,"LiftElevator"));
		sWaypoints.add(new Waypoint(180,48,45,137));
		sWaypoints.add(new Waypoint(270,75,0,137,"ChuteMe"));
		sWaypoints.add(PathAdapter.getAdaptedRightScaleWaypoint(new Waypoint(300,84,0,137)));

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