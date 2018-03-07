package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_4cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightRightFromRightStep6 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(283,88,0,0));
		sWaypoints.add(new Waypoint(245,77,20,60));
		sWaypoints.add(new Waypoint(240,133,15,80));
		sWaypoints.add(new Waypoint(228,158,0,60));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(283, 88), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA: [{"position":{"x":283,"y":75},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":245,"y":77},"speed":60,"radius":20,"marker":"","comment":""},{"position":{"x":240,"y":133},"speed":80,"radius":15,"marker":"","comment":""},{"position":{"x":223,"y":142},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightRightFromRightStep6
}