package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftLeft_2cube;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftLeftFromRight_2cubeStep2 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(PathAdapter.getAdaptedLeftScaleWaypoint(new Waypoint(290,257,0,0)));
		sWaypoints.add(new Waypoint(265,257,15,23,"PreparePickupCube"));
		sWaypoints.add(PathAdapter.getAdaptedLeftSwitchWaypoint(new Waypoint(232,248,0,23)));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(290, 257), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA: [{"position":{"x":290,"y":257},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":265,"y":257},"speed":20,"radius":15,"marker":"PreparePickupCube","comment":""},{"position":{"x":232,"y":248},"speed":20,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftLeftFromRight_2cubeStep2
}