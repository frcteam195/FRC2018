package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.LeftLeft_3cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftLeftFromLeftStep4 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(212,230,0,0));
		sWaypoints.add(new Waypoint(220,241,10,60));
		sWaypoints.add(new Waypoint(230,255,0,60));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(212, 230), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
	// WAYPOINT_DATA: [{"position":{"x":212,"y":230},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":220,"y":241},"speed":60,"radius":10,"marker":"","comment":""},{"position":{"x":230,"y":255},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: LeftLeftFromLeftStep4
}