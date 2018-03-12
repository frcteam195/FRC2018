package org.usfirst.frc.team195.robot.Autonomous.Paths.StartFromLeft.LeftLeft_3cube;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class LeftLeftFromLeftStep5 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(246,258,0,0));
        sWaypoints.add(new Waypoint(248,220,15,60));
        sWaypoints.add(new Waypoint(230,216,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(246, 258), Rotation2d.fromDegrees(0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":246,"y":258},"speed":0,"radius":0,"marker":"","comment":""},{"position":{"x":248,"y":220},"speed":60,"radius":15,"marker":"","comment":""},{"position":{"x":230,"y":216},"speed":60,"radius":0,"marker":"","comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: LeftLeftFromLeftStep5
}