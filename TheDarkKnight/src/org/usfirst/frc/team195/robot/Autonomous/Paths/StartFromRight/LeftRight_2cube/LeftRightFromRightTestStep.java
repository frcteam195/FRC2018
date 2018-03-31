package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftRight_2cube;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftRightFromRightTestStep implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(PathAdapter.getAdaptedRightScaleWaypoint(new Waypoint(286,76,0,0)));
		sWaypoints.add(new Waypoint(242,70,30,30));
		sWaypoints.add(new Waypoint(242,139,10,60,"PreparePickupCube"));
		sWaypoints.add(new Waypoint(252,200,0,30));
		sWaypoints.add(new Waypoint(248,204,0,30));
		sWaypoints.add(PathAdapter.getAdaptedLeftSwitchWaypoint(new Waypoint(228,204,0,30)));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(286, 76), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA: [{"position":{"x":286,"y":76},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":242,"y":70},"speed":30,"radius":30,"marker":"PreparePickupCube","comment":""},{"position":{"x":242,"y":139},"speed":60,"radius":10,"marker":"StartIntake","comment":""},{"position":{"x":252,"y":200},"speed":30,"radius":0,"marker":"","comment":""},{"position":{"x":248,"y":204},"speed":30,"radius":0,"marker":"","comment":""},{"position":{"x":230,"y":204},"speed":30,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftRightFromRightTestStep
}