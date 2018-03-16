package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftLeft_3cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftLeftFromRightStep1 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,48,0,0));
		sWaypoints.add(new Waypoint(165,45,30,140));
		sWaypoints.add(new Waypoint(238,77,30,100));
		sWaypoints.add(new Waypoint(238,220,15,120,"PreparePlaceCube"));
		sWaypoints.add(new Waypoint(258,260,15,100));
		sWaypoints.add(new Waypoint(290,257,0,80));

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
	// WAYPOINT_DATA: [{"position":{"x":20,"y":48},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":165,"y":45},"speed":140,"radius":30,"marker":"","comment":""},{"position":{"x":238,"y":77},"speed":100,"radius":30,"marker":"","comment":""},{"position":{"x":238,"y":220},"speed":120,"radius":15,"marker":"PreparePlaceCube","comment":""},{"position":{"x":258,"y":260},"speed":100,"radius":15,"marker":"","comment":""},{"position":{"x":290,"y":257},"speed":80,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: LeftLeftFromRightStep1
}