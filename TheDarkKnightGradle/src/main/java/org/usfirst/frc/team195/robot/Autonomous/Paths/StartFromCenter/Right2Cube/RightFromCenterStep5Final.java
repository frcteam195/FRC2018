package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromCenter.Right2Cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightFromCenterStep5Final implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(50,163,0,0));
		sWaypoints.add(new Waypoint(67,150,15,60));
		sWaypoints.add(new Waypoint(85,112,20,60,"ArmDown"));
		sWaypoints.add(new Waypoint(116,108,0,60));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(50, 163), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA: [{"position":{"x":50,"y":163},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":67,"y":150},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":85,"y":112},"speed":60,"radius":20,"marker":"ArmDown","comment":""},{"position":{"x":116,"y":112},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightFromCenterStep5Final
}