package org.usfirst.frc.team195.robot.Autonomous.Paths;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class SamplePath implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
//		sWaypoints.add(new Waypoint(20,260,0,0));
//		sWaypoints.add(new Waypoint(220,260,25,60));
//		sWaypoints.add(new Waypoint(236,225,15,60));
//		sWaypoints.add(new Waypoint(242,180,0,60));

//		sWaypoints.add(new Waypoint(20,260,0,0));
//		sWaypoints.add(new Waypoint(100,260,0,60));

//		sWaypoints.add(new Waypoint(20,260,0,0));
//		sWaypoints.add(new Waypoint(60,260,30,60));
//		sWaypoints.add(new Waypoint(60,215,0,60));

//		sWaypoints.add(new Waypoint(20,45,0,0));
//		sWaypoints.add(new Waypoint(148,45,30,140));
//		sWaypoints.add(new Waypoint(225,77,30,140));
//		sWaypoints.add(new Waypoint(282,84,0,140));

		sWaypoints.add(new Waypoint(20,45,0,0));
		sWaypoints.add(new Waypoint(165,45,30,140));
		sWaypoints.add(new Waypoint(238,77,30,100));
		sWaypoints.add(new Waypoint(238,220,15,120));
		sWaypoints.add(new Waypoint(258,260,15,100));
		sWaypoints.add(new Waypoint(290,257,0,80));

//		sWaypoints.add(new Waypoint(20,45,0,0));
//		sWaypoints.add(new Waypoint(116,51,15,140));
//		sWaypoints.add(new Waypoint(376,38,15,140));
//		sWaypoints.add(new Waypoint(574,61,30,140));
//		sWaypoints.add(new Waypoint(591,199,30,140));
//		sWaypoints.add(new Waypoint(559,279,30,140));
//		sWaypoints.add(new Waypoint(444,284,15,140));
//		sWaypoints.add(new Waypoint(229,289,15,140));
//		sWaypoints.add(new Waypoint(90,281,15,140));
//		sWaypoints.add(new Waypoint(53,255,15,140));
//		sWaypoints.add(new Waypoint(58,131,15,140));
//		sWaypoints.add(new Waypoint(61,68,15,140));
//		sWaypoints.add(new Waypoint(20,45,0,60));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(20, 45), Rotation2d.fromDegrees(180));
	}

	@Override

	public boolean isReversed() {
		return true;
	}
	// WAYPOINT_DATA:
	// [{"position":{"x":16,"y":160},"speed":0,"radius":0,"comment":""},{"position":{"x":90,"y":160},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: StartToCenterGearBlue
}
