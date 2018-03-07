package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_4cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightRightFromRightStep1 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,48,0,0));
		sWaypoints.add(new Waypoint(60,48,0,140));
		sWaypoints.add(new Waypoint(180,48,45,140));
		sWaypoints.add(new Waypoint(286,86,0,100));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(20, 48), Rotation2d.fromDegrees(180));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
	// WAYPOINT_DATA: [{"position":{"x":20,"y":48},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":60,"y":48},"speed":140,"radius":0,"marker":"","comment":""},{"position":{"x":180,"y":48},"speed":140,"radius":45,"marker":"","comment":""},{"position":{"x":286,"y":76},"speed":100,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: UntitledPath
}