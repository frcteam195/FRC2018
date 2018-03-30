package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromCenter.Right2Cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightFromCenterStep2 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(120,110,0,0));
		sWaypoints.add(new Waypoint(85,112,20,60));
		sWaypoints.add(new Waypoint(67,150,15,60,"PreparePickupCube"));
		sWaypoints.add(new Waypoint(50,163,0,60));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(120, 110), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
	// WAYPOINT_DATA: [{"position":{"x":120,"y":110},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":85,"y":112},"speed":60,"radius":20,"marker":"","comment":""},{"position":{"x":67,"y":150},"speed":60,"radius":15,"marker":"PreparePickupCube","comment":""},{"position":{"x":50,"y":163},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: RightFromCenterStep2
}