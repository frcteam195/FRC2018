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
		sWaypoints.add(new Waypoint(250,230,15,60));
		sWaypoints.add(new Waypoint(234,234,0,20));

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
	// WAYPOINT_DATA: [{"position":{"x":176,"y":74},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":176,"y":56},"speed":60,"radius":10,"marker":"","comment":""},{"position":{"x":209,"y":40},"speed":60,"radius":15,"marker":"PreparePickupCube","comment":""},{"position":{"x":248,"y":56},"speed":60,"radius":10,"marker":"","comment":""},{"position":{"x":248,"y":74},"speed":60,"radius":0,"marker":"","comment":""},{"position":{"x":248,"y":150},"speed":60,"radius":0,"marker":"StartIntake","comment":""},{"position":{"x":248,"y":190},"speed":60,"radius":10,"marker":"","comment":""},{"position":{"x":250,"y":230},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":234,"y":234},"speed":20,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightLeftFromRightStep2
}