package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightLeft_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightLeftFromRightStep2 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(176,74,0,0));
		sWaypoints.add(new Waypoint(176,56,10,60));
		sWaypoints.add(new Waypoint(209,40,15,60,"PreparePickupCube"));
		sWaypoints.add(new Waypoint(248,56,10,60));
		sWaypoints.add(new Waypoint(248,74,0,60));
		sWaypoints.add(new Waypoint(248,150,0,60,"StartIntake"));
		sWaypoints.add(new Waypoint(248,190,10,60));
		sWaypoints.add(new Waypoint(250,214,10,60));
		sWaypoints.add(new Waypoint(235,223,0,20));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(176, 74), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}