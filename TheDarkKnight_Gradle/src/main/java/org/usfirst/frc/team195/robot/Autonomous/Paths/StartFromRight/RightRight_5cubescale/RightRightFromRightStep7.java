package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_5cubescale;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightRightFromRightStep7 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(286,78,0,0));
		sWaypoints.add(new Waypoint(245,77,20,60));
		sWaypoints.add(new Waypoint(240,133,15,80));
		sWaypoints.add(new Waypoint(228,158,0,60));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(286, 78), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}