package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightLeft_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightLeftFromRightStep1 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,48,0,0));
		sWaypoints.add(new Waypoint(40,48,15,140));
		sWaypoints.add(new Waypoint(70,35,15,120));
		sWaypoints.add(new Waypoint(130,35,0,100,"PreparePlaceCube"));
		sWaypoints.add(new Waypoint(162,35,25,100));
		sWaypoints.add(new Waypoint(176,74,0,80));

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
	// WAYPOINT_DATA: [{"position":{"x":20,"y":48},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":40,"y":48},"speed":140,"radius":15,"marker":"","comment":""},{"position":{"x":70,"y":35},"speed":120,"radius":15,"marker":"","comment":""},{"position":{"x":130,"y":35},"speed":100,"radius":0,"marker":"PreparePlaceCube","comment":""},{"position":{"x":162,"y":35},"speed":100,"radius":25,"marker":"","comment":""},{"position":{"x":176,"y":73},"speed":80,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: RightLeftFromRightStep1
}