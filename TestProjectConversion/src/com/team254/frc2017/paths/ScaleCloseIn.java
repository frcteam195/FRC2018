package com.team254.frc2017.paths;

import com.team254.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;
import com.team254.lib.util.math.RigidTransform2d;
import com.team254.lib.util.math.Rotation2d;
import com.team254.lib.util.math.Translation2d;

import java.util.ArrayList;

/**
 * Path from the blue alliance wall to the blue center peg.
 *
 * Used in CenterGearToShootBlue
 *
 * @see CenterGearToShootBlue
 * @see PathContainer
 */
public class ScaleCloseIn implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(new Waypoint(325,285,0,0));
		sWaypoints.add(new Waypoint(325,265,0,60));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(325, 285), Rotation2d.fromDegrees(90));
	}

	@Override

	public boolean isReversed() {
		return false;
	}
	// WAYPOINT_DATA:
	// [{"position":{"x":16,"y":160},"speed":0,"radius":0,"comment":""},{"position":{"x":90,"y":160},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: StartToCenterGearBlue
}
