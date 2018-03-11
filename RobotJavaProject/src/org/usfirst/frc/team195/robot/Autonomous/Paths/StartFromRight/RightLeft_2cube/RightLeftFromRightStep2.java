package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromRight.RightLeft_2cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class RightLeftFromRightStep2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(168,30,0,0));
        sWaypoints.add(new Waypoint(168,64,0,20));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(168, 30), Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":168,"y":50},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":168,"y":72},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: RightLeftFromRightStep2
}