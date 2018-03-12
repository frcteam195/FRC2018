package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightLeft_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightLeftFromRightStep2 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(176,73,0,0));
		sWaypoints.add(new Waypoint(176,56,15,60));
		sWaypoints.add(new Waypoint(209,40,15,60,"PreparePickupCube"));
		sWaypoints.add(new Waypoint(242,56,15,60));
		sWaypoints.add(new Waypoint(242,73,0,60));
		sWaypoints.add(new Waypoint(242,190,10,60,"StartIntake"));
		sWaypoints.add(new Waypoint(246,216,10,60));
		sWaypoints.add(new Waypoint(234,226,0,60));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(176, 73), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA: [{"position":{"x":176,"y":73},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":176,"y":56},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":209,"y":40},"speed":60,"radius":15,"marker":"PreparePickupCube","comment":""},{"position":{"x":242,"y":56},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":242,"y":73},"speed":60,"radius":0,"marker":"","comment":""},{"position":{"x":242,"y":190},"speed":60,"radius":10,"marker":"StartIntake","comment":""},{"position":{"x":246,"y":216},"speed":60,"radius":10,"marker":"","comment":""},{"position":{"x":234,"y":226},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightLeftFromRightStep2
}