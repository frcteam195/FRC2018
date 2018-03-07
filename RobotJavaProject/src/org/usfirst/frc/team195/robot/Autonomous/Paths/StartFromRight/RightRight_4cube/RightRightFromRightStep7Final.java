package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_4cube;


import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightRightFromRightStep7Final implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(223,142,0,0));
		sWaypoints.add(new Waypoint(240,133,15,60));
		sWaypoints.add(new Waypoint(245,77,20,80));
		sWaypoints.add(new Waypoint(287,79,0,60));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(223, 142), Rotation2d.fromDegrees(180));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
	// WAYPOINT_DATA: [{"position":{"x":223,"y":142},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":240,"y":133},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":245,"y":77},"speed":80,"radius":20,"marker":"","comment":""},{"position":{"x":287,"y":79},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: RightRightFromRightStep7Final
}