package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromCenter.Right3Cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightFromCenterStep1 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,157,0,0));
		sWaypoints.add(new Waypoint(50,157,25,120));
		sWaypoints.add(new Waypoint(80,114,25,100,"ArmDown"));
		sWaypoints.add(new Waypoint(120,112,0,80));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(20, 157), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA: [{"position":{"x":20,"y":157},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":50,"y":157},"speed":120,"radius":25,"marker":"","comment":""},{"position":{"x":80,"y":114},"speed":100,"radius":25,"marker":"ArmDown","comment":""},{"position":{"x":120,"y":112},"speed":80,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightFromCenterStep1
}