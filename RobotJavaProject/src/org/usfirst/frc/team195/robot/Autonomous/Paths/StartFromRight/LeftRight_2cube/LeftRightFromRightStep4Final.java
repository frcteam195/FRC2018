package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.LeftRight_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftRightFromRightStep4Final implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(236,230,0,0));
        sWaypoints.add(new Waypoint(218,228,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(236, 230), Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
	// WAYPOINT_DATA: [{"position":{"x":236,"y":230},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":218,"y":228},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: LeftRightFromRightStep4Final
}