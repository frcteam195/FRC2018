package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftLeft_2cube;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;
public class LeftLeftFromRight_2cubeStep1 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(20,48,0,0));
		sWaypoints.add(new Waypoint(165,45,30,130));
		sWaypoints.add(new Waypoint(238,77,30,120));
		sWaypoints.add(new Waypoint(238,220,15,100,"PreparePlaceCube"));
		sWaypoints.add(new Waypoint(258,260,15,70));
		sWaypoints.add(PathAdapter.getAdaptedLeftScaleWaypoint(new Waypoint(290,257,0,50)));

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
	// WAYPOINT_DATA: [{"position":{"x":20,"y":48},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":165,"y":45},"speed":80,"radius":30,"marker":"","comment":""},{"position":{"x":238,"y":77},"speed":80,"radius":30,"marker":"","comment":""},{"position":{"x":238,"y":220},"speed":100,"radius":15,"marker":"PreparePlaceCube","comment":""},{"position":{"x":258,"y":260},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":290,"y":257},"speed":40,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: LeftLeftFromRight_2cubeStep1
}