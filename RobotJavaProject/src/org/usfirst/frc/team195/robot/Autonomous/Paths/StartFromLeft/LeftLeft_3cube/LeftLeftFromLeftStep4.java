package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.LeftLeft_3cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftLeftFromLeftStep4 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(219,241,0,0));
        sWaypoints.add(new Waypoint(239,241,15,60));
        sWaypoints.add(new Waypoint(246,258,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(219, 241), Rotation2d.fromDegrees(0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":219,"y":241},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":239,"y":241},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":246,"y":258},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftLeftFromLeftStep4
}