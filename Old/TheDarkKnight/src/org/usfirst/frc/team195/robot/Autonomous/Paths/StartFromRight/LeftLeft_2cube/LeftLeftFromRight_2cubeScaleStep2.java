package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftLeft_2cube;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftLeftFromRight_2cubeScaleStep2 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(PathAdapter.getAdaptedLeftSwitchWaypoint(new Waypoint(232,248,0,0)));
		sWaypoints.add(new Waypoint(265,257,15,60,"PreparePlaceCube"));
		sWaypoints.add(PathAdapter.getAdaptedLeftScaleWaypoint(new Waypoint(290,248,0,60)));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(232, 248), Rotation2d.fromDegrees(0));
	}

	@Override
	public boolean isReversed() {
		return true;
	}
	// WAYPOINT_DATA: [{"position":{"x":232,"y":248},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":265,"y":257},"speed":20,"radius":15,"marker":"PreparePlaceCube","comment":""},{"position":{"x":290,"y":248},"speed":20,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: LeftLeftFromRight_2cubeScaleStep2
}