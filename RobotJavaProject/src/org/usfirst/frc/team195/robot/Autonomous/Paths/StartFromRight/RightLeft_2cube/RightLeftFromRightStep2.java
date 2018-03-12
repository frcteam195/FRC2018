package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightLeft_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightLeftFromRightStep2 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(176,74,0,0));
		sWaypoints.add(new Waypoint(176,56,10,80));
		sWaypoints.add(new Waypoint(209,40,15,80,"PreparePickupCube"));
		sWaypoints.add(new Waypoint(242,56,10,80));
		sWaypoints.add(new Waypoint(242,74,0,80));
		sWaypoints.add(new Waypoint(242,150,0,120,"StartIntake"));
		sWaypoints.add(new Waypoint(242,190,10,30));
		sWaypoints.add(new Waypoint(268,264,30,30));
		sWaypoints.add(new Waypoint(227,242,0,30));

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
	// WAYPOINT_DATA: [{"position":{"x":176,"y":74},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":176,"y":56},"speed":80,"radius":10,"marker":"","comment":""},{"position":{"x":209,"y":40},"speed":80,"radius":15,"marker":"PreparePickupCube","comment":""},{"position":{"x":242,"y":56},"speed":80,"radius":10,"marker":"","comment":""},{"position":{"x":242,"y":74},"speed":80,"radius":0,"marker":"","comment":""},{"position":{"x":242,"y":150},"speed":120,"radius":0,"marker":"StartIntake","comment":""},{"position":{"x":242,"y":190},"speed":30,"radius":10,"marker":"","comment":""},{"position":{"x":268,"y":264},"speed":30,"radius":30,"marker":"","comment":""},{"position":{"x":227,"y":242},"speed":30,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightLeftFromRightStep2
}