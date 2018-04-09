package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightRight_4cube;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightRightFromRightCube3Ready44 implements PathContainer {

	@Override
	public Path buildPath() {
		ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
		sWaypoints.add(PathAdapter.getAdaptedRightScaleWaypoint(new Waypoint(286,86,0,0)));
		sWaypoints.add(new Waypoint(262,81,15,50));
		sWaypoints.add(new Waypoint(239,100,10,50));
		sWaypoints.add(new Waypoint(232,118,0,50));

		return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
		return new RigidTransform2d(new Translation2d(286, 86), Rotation2d.fromDegrees(180));
	}

	@Override
	public boolean isReversed() {
		return false;
	}
}