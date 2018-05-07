package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_5cubescale;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightRightFromRightGOBACK implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(PathAdapter.getAdaptedRightSwitchWaypoint(new Waypoint(228,90,0,0)));
		sWaypoints.add(new Waypoint(257,82,15,60,"PreparePlaceCube"));
		sWaypoints.add(PathAdapter.getAdaptedRightScaleWaypoint(new Waypoint(286,89,0,60)));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(228, 90), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
	// WAYPOINT_DATA: [{"position":{"x":228,"y":90},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":257,"y":82},"speed":20,"radius":15,"marker":"PreparePlaceCube","comment":""},{"position":{"x":286,"y":89},"speed":20,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: RightRightFromRightGOBACK
}